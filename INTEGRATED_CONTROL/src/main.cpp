// =====================================================================
// MediTilt Pro - FULLY MERGED Controller (ESP32)
//
// PRIORITY ORDER (highest to lowest):
//   1. HARDWARE REMOTE  (kill switch > homing > manual jog)
//   2. NEXTION DISPLAY  (physical touch H-commands + USB blink nav)
//   3. WEB DASHBOARD    (WiFi AP + REST-ish /api)
//
// ===================== READ BEFORE FLASHING =========================
// (1) MOTOR DIRECTION POLARITY CONFLICT BETWEEN SOURCE FILES:
//     The remote-only sketch drove DIR_PIN=LOW as "increasing" (this is
//     the version with the backlash-compensation / creep-speed fixes).
//     The Nextion+web sketch drove DIR_PIN=HIGH as "increasing" - the
//     OPPOSITE polarity. This merge keeps the remote sketch's polarity
//     (LOW = increasing) since it's the more recently debugged version.
//     VERIFY ON THE BENCH: command a small move from the web dashboard
//     and from Nextion and confirm the bed actually moves the direction
//     you expect before trusting either of those paths.
//
// (2) GPIO36 (encoder ADC1) + WiFi AP running concurrently is a
//     previously-diagnosed noise source for you (encoder count
//     corruption -> "down" direction failures). Merging in the web
//     dashboard means the AP is now always on next to the encoder read.
//     A 3-sample median filter was added to readEncoder() as a
//     mitigation, but this is a patch, not a guarantee - watch
//     printDebugStatus() output for spurious rotation jumps under load.
// =====================================================================

#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "calibration_table.h"

// Set true to test priority/arbitration logic without motor power connected.
#define SIMULATE_MOTOR false

// ===================================================================
// FORWARD DECLARATIONS (required for PlatformIO .cpp builds)
// ===================================================================
void handleRoot();
void handleAPI();
void setBedTarget(float targetAngle);
long rollToRotations(float targetRoll);
long applyBacklashCompensation(long rawTarget);
void updateMotorControl();
void setupMotor();
void readEncoder();
void setupMPU();
void readMPU();
void calibrateMPU();
String getCurrentButtonID();
void endNextionCmd();
void highlightCurrentButton(int colorCode);
void clearAllButtonsOnPage();
void handleNext();
void handleSelect();
void interpretBedCommand(String cmd);
void applyTouchModeForCurrentPage();
void printDebugStatus();
byte read74HC165();
void updateRemoteLED(bool isCommandActive);
void setupLegMotor();
void legMotorStop();
void legMotorDriveUp();
void legMotorDriveDown();
void commandLegManual(int dir);
void commandHeadManual(int dir);
void commandTiltSim(int dir);
void haltAllMotorsHard();
void pollRemote();
void updateHomingSimulation();
bool remoteHasPriority();
bool nextionHasPriority();
void markRemoteActive();
void markNextionActive();

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ===================================================================
// PIN MAP (superset of both source sketches - verified no overlaps)
// ===================================================================
// I2C (MPU6050):            SDA 4,  SCL 5
// Nextion:                  Serial2 RX 16, TX 17
// AS5600 encoder:           ADC1 GPIO36 (must stay on ADC1 - ADC2 shares
//                           the WiFi radio and is unusable while the AP
//                           is up)
// Torso/head Cytron motor:  PWM 12, DIR 13
// Leg/foot motor (digital): PWM 25, DIR 26
// Remote 74HC165:           LATCH 27, CLOCK 14, DATA 19
const int REMOTE_LATCH_PIN = 27;
const int REMOTE_CLOCK_PIN = 14;
const int REMOTE_DATA_PIN  = 19; // INPUT_PULLDOWN - defaults to 0 if tether breaks

#define LEG_MOTOR_PWM_PIN 25
#define LEG_MOTOR_DIR_PIN 26

#define MOTOR_PWM_PIN 12
#define MOTOR_DIR_PIN 13

#define ENCODER_PIN 36

// ===================================================================
// WIFI / WEB DASHBOARD (lowest priority)
// ===================================================================
const char* ssid = "MediTilt_Pro_Net";
const char* password = "clinicalcontrol"; // min 8 chars

WebServer server(80);

int headAngle = 0;
int footAngle = 0;
int leftTilt = 0;
int rightTilt = 0;

// false = Nextion physical touch active; true = blink mode (USB bytes
// drive nav, physical touch locked via tsw).
bool blinkModeEnabled = true;

// ===================================================================
// PRIORITY ARBITRATION
// ===================================================================
// Remote: highest priority. Any remote activity (manual jog on ANY axis,
// or an active homing sequence, or the kill switch) locks out both
// Nextion and web for this many ms after the activity stops, to avoid
// a lower-priority source snatching the motor the instant a button is
// released.
unsigned long lastRemoteActivityMillis = 0;
const unsigned long REMOTE_PRIORITY_COOLDOWN_MS = 2000;

// Nextion: mid priority. Locks out web for this many ms after any
// Nextion activity (physical H-command or blink nav triggering one).
unsigned long lastNextionActivityMillis = 0;
const unsigned long NEXTION_PRIORITY_COOLDOWN_MS = 8000; // long for easy bench testing; tighten for production

volatile bool emergencyStopActive = false;
volatile bool headManualJogActive = false;

bool isHomingActive = false;
int homingStage = 0; // 0 Idle, 1 Tilt(sim), 2 Leg(sim), 3 Head(physical)
unsigned long homingTimer = 0;

void markRemoteActive() { lastRemoteActivityMillis = millis(); }
void markNextionActive() { lastNextionActivityMillis = millis(); }

// True whenever the remote outranks everything else and Nextion/web
// commands to the torso motor must be refused.
bool remoteHasPriority() {
  if (emergencyStopActive) return true;
  if (headManualJogActive) return true;
  if (isHomingActive) return true;
  return (millis() - lastRemoteActivityMillis) < REMOTE_PRIORITY_COOLDOWN_MS;
}

// True whenever Nextion outranks web (remote is assumed already clear
// by the caller - check remoteHasPriority() first).
bool nextionHasPriority() {
  return (millis() - lastNextionActivityMillis) < NEXTION_PRIORITY_COOLDOWN_MS;
}

// Remote button-state edge tracking
int lastHeadState = 0;
int lastLegState = 0;
int lastTiltState = 0;
bool lastKillState = false;

unsigned long lastRemoteBlinkTime = 0;
bool remoteLedState = HIGH;

unsigned long lastDebugPrintMillis = 0;
const unsigned long DEBUG_PRINT_INTERVAL_MS = 250;

// ===================================================================
// WEB DASHBOARD HTML (unchanged from source - self-contained page)
// ===================================================================
const char html_page[] PROGMEM = R"=====(
<!doctype html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover" />
    <title>MediTilt Pro — Clinical Control Station</title>
    <style>
      @import url("https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@400;500;600;700;800&display=swap");
      :root {
        --bg: #0b0f17; --panel: #131924; --card: #1a2232; --border: rgba(255, 255, 255, 0.06);
        --accent: #00f2fe; --accent-glow: rgba(0, 242, 254, 0.2); --mint: #05f2c7;
        --text: #f8fafc; --text-muted: #64748b; --radius-lg: 1rem; --radius-sm: 0.5rem;
        --transition: 0.25s cubic-bezier(0.4, 0, 0.2, 1); --drag-motion: 0.1s linear; --smooth-motion: 2.5s cubic-bezier(0.1, 0.8, 0.25, 1);
      }
      * { margin: 0; padding: 0; box-sizing: border-box; font-family: "Plus Jakarta Sans", sans-serif; -webkit-font-smoothing: antialiased; }
      body {
        background-color: var(--bg);
        background-image: radial-gradient(at 0% 0%, rgba(0, 242, 254, 0.05) 0px, transparent 50%), radial-gradient(at 100% 100%, rgba(5, 242, 199, 0.03) 0px, transparent 50%);
        min-height: 100vh; display: flex; align-items: center; justify-content: center; padding: 1rem; color: var(--text);
      }
      .dashboard-container { width: 100%; max-width: 1100px; background: var(--panel); border: 1px solid var(--border); border-radius: var(--radius-lg); padding: 1.5rem; box-shadow: 0 40px 80px -20px rgba(0, 0, 0, 0.7); display: flex; flex-direction: column; gap: 1.5rem; }
      header { display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid var(--border); padding-bottom: 1rem; }
      .brand h1 { font-size: 1.5rem; font-weight: 800; letter-spacing: -0.5px; background: linear-gradient(120deg, #fff, #94a3b8); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
      .brand p { font-size: 0.7rem; font-weight: 600; text-transform: uppercase; letter-spacing: 2px; color: var(--mint); margin-top: 0.2rem; }
      .status-badge { font-size: 0.7rem; font-weight: 700; background: rgba(5, 242, 199, 0.1); color: var(--mint); padding: 0.4rem 0.8rem; border-radius: 2rem; border: 1px solid rgba(5, 242, 199, 0.2); }
      .status-banner { display: none; background: rgba(255, 90, 90, 0.12); color: #ff8080; border: 1px solid rgba(255, 90, 90, 0.3); border-radius: var(--radius-sm); padding: 0.65rem 1rem; font-size: 0.8rem; font-weight: 700; text-align: center; }
      .status-banner.active { display: block; }
      .main-grid { display: grid; grid-template-columns: 1fr; gap: 1.25rem; }
      @media (min-width: 900px) { .main-grid { grid-template-columns: 1.2fr 1fr; } }
      .section-column { display: flex; flex-direction: column; gap: 1.25rem; }
      .panel-card { background: var(--card); border: 1px solid var(--border); border-radius: var(--radius-lg); padding: 1.25rem; }
      .panel-card h2 { font-size: 0.85rem; font-weight: 700; text-transform: uppercase; letter-spacing: 1.5px; color: var(--text-muted); margin-bottom: 1rem; display: flex; align-items: center; gap: 0.5rem; }
      .visual-viewport { display: flex; flex-direction: column; gap: 1rem; background: rgba(0, 0, 0, 0.2); padding: 1rem; border-radius: var(--radius-sm); border: 1px solid rgba(255, 255, 255, 0.02); }
      .view-container { position: relative; height: 90px; width: 100%; display: flex; align-items: flex-end; justify-content: center; overflow: hidden; user-select: none; }
      @media (min-width: 900px) { .view-container { height: 120px; } }
      .base-chassis { position: absolute; bottom: 10px; width: 90%; height: 6px; background: #334155; border-radius: 4px; }
      .chassis-post { position: absolute; bottom: 16px; width: 8px; height: 25px; background: #475569; }
      .chassis-post.left { left: 8%; border-radius: 4px 4px 0 0; }
      .chassis-post.right { right: 8%; border-radius: 4px 4px 0 0; }
      .mattress-linkage { position: absolute; bottom: 22px; width: 78%; height: 20px; display: flex; align-items: flex-end; }
      .center-pelvic-node { width: 26%; height: 14px; background: #1e293b; border: 1px solid #334155; border-radius: 3px; margin: 0 2px; z-index: 2; }
      .head-node { width: 37%; height: 14px; background: linear-gradient(90deg, var(--accent), var(--mint)); border-radius: 3px; box-shadow: 0 0 15px var(--accent-glow); transition: transform var(--smooth-motion); will-change: transform; transform-origin: right bottom; cursor: grab; touch-action: none; }
      .head-node:active { cursor: grabbing; }
      .foot-node-container { width: 37%; height: 14px; position: relative; transform-origin: left bottom; transition: transform var(--smooth-motion); will-change: transform; cursor: grab; touch-action: none; }
      .foot-node-container:active { cursor: grabbing; }
      .foot-node-thigh { width: 50%; height: 100%; background: linear-gradient(90deg, var(--accent), #02decb); border-radius: 3px 0 0 3px; position: absolute; left: 0; top: 0; }
      .foot-node-flap { width: 50%; height: 100%; background: linear-gradient(90deg, #02decb, var(--mint)); border-radius: 0 3px 3px 0; position: absolute; left: 50%; top: 0; transform-origin: left top; transition: transform var(--smooth-motion); will-change: transform; box-shadow: 3px 0 10px var(--accent-glow); }
      .transverse-container { align-items: center; }
      .transverse-deck { position: relative; width: 65%; height: 16px; }
      .deck-wing-l, .deck-wing-r { position: absolute; width: 49.5%; height: 12px; background: linear-gradient(135deg, var(--accent), var(--mint)); box-shadow: 0 0 15px var(--accent-glow); transition: transform var(--smooth-motion); will-change: transform; cursor: grab; touch-action: none; }
      .deck-wing-l:active, .deck-wing-r:active { cursor: grabbing; }
      .deck-wing-l { left: 0; border-radius: 4px 0 0 4px; transform-origin: right bottom; transform: rotate(0deg); }
      .deck-wing-r { right: 0; border-radius: 0 4px 4px 0; transform-origin: left bottom; transform: rotate(0deg); }
      .dragging-active { transition: transform var(--drag-motion) !important; }
      .pivot-pin { position: absolute; bottom: -4px; left: 50%; transform: translateX(-50%); width: 10px; height: 10px; background: #fff; border-radius: 50%; box-shadow: 0 0 10px #fff; z-index: 5; }
      .telemetry-output { display: flex; justify-content: space-between; font-size: 0.75rem; font-weight: 700; color: var(--text); background: rgba(255, 255, 255, 0.03); padding: 0.5rem 0.75rem; border-radius: var(--radius-sm); border: 1px solid var(--border); }
      .telemetry-output span { color: var(--accent); }
      .control-block { display: flex; flex-direction: column; gap: 1rem; }
      .control-subgroup { background: rgba(0, 0, 0, 0.15); padding: 1rem; border-radius: var(--radius-sm); border: 1px solid var(--border); display: flex; flex-direction: column; gap: 0.75rem; }
      .control-subgroup h3 { font-size: 0.75rem; color: var(--text-muted); text-transform: uppercase; letter-spacing: 0.5px; }
      .button-row { display: flex; gap: 0.4rem; flex-wrap: wrap; }
      .btn { background: #222d3d; border: 1px solid var(--border); border-radius: var(--radius-sm); padding: 0.65rem 0.85rem; font-weight: 600; font-size: 0.8rem; color: var(--text); cursor: pointer; transition: var(--transition); display: inline-flex; align-items: center; justify-content: center; gap: 0.4rem; flex: 1 1 auto; }
      .btn:hover { background: #2a374a; border-color: rgba(255, 255, 255, 0.15); }
      .btn:active { transform: scale(0.97); }
      .btn.preset { min-width: 40px; flex: 1; }
      .btn.preset.active { background: linear-gradient(135deg, var(--accent), var(--mint)); color: var(--bg); font-weight: 700; border-color: transparent; box-shadow: 0 0 15px var(--accent-glow); }
      .header-toggle-wrap { display: flex; justify-content: space-between; align-items: center; margin-bottom: 1rem; }
      .switch-box { display: flex; align-items: center; gap: 0.75rem; font-weight: 600; font-size: 0.85rem; }
      .switch-box span.active-state { color: var(--text-muted); }
      input:checked ~ span.active-state { color: var(--accent); }
      .switch { position: relative; width: 44px; height: 22px; }
      .switch input { opacity: 0; width: 0; height: 0; }
      .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background: #2d394a; border-radius: 2rem; transition: var(--transition); }
      .slider:before { content: ""; position: absolute; height: 16px; width: 16px; left: 3px; bottom: 3px; background: #fff; border-radius: 50%; transition: var(--transition); }
      input:checked + .slider { background: var(--accent); }
      input:checked + .slider:before { transform: translateX(22px); background: var(--bg); }
      .blink-terminal { background: rgba(0, 0, 0, 0.3); border-radius: var(--radius-sm); border: 1px dashed rgba(0, 242, 254, 0.25); padding: 1rem; display: none; flex-direction: column; gap: 1rem; }
      .blink-terminal.active { display: flex; }
      .blink-chip { background: #19202e; border: 1px solid var(--border); border-radius: 4px; padding: 0.5rem; font-size: 0.75rem; font-weight: 600; color: var(--text-muted); text-align: center; transition: var(--transition); }
      .blink-chip.current { background: rgba(0, 242, 254, 0.08); color: var(--accent); border-color: var(--accent); font-weight: 700; }
      .selection-focus { font-size: 0.8rem; font-weight: 600; color: var(--text-muted); }
      .selection-focus span { background: var(--accent); color: var(--bg); font-weight: 700; padding: 0.2rem 0.6rem; border-radius: 3px; margin-left: 0.4rem; }
      @media (max-width: 600px) { body { padding: 0.5rem; } .dashboard-container { padding: 1rem; gap: 1rem; } header { flex-direction: column; align-items: flex-start; gap: 0.75rem; } .status-badge { align-self: flex-start; } }
    </style>
  </head>
  <body>
    <div class="dashboard-container">
      <header>
        <div class="brand"><h1>MediTilt Pro</h1><p>Decubitus Prevention System</p></div>
        <div class="status-badge">System Core Online</div>
      </header>
      <div id="statusBanner" class="status-banner"></div>
      <div class="main-grid">
        <div class="section-column">
          <div class="panel-card control-block">
            <h2>📐 Head & Foot (Sagittal Kinematics)</h2>
            <div class="visual-viewport">
              <div class="view-container">
                <div class="base-chassis"></div><div class="chassis-post left"></div><div class="chassis-post right"></div>
                <div class="mattress-linkage">
                  <div class="head-node" id="headSeg"></div><div class="center-pelvic-node"></div>
                  <div class="foot-node-container" id="footSegContainer"><div class="foot-node-thigh"></div><div class="foot-node-flap" id="footFlapSeg"></div></div>
                </div>
              </div>
              <div class="telemetry-output">Position Monitor <span id="sagDisplay">Head 0° · Foot 0°</span></div>
            </div>
            <div class="control-subgroup">
              <h3>Head Section Adjustment</h3>
              <div class="button-row" id="headPresetGroup">
                <button class="btn preset" data-angle="0">0°</button><button class="btn preset" data-angle="15">15°</button><button class="btn preset" data-angle="30">30°</button><button class="btn preset" data-angle="45">45°</button><button class="btn preset" data-angle="60">60°</button>
              </div>
              <div class="button-row"><button class="btn" id="headUpBtn">▲ Raise Head</button><button class="btn" id="headDownBtn">▼ Lower Head</button></div>
            </div>
            <div class="control-subgroup">
              <h3>Foot Section Adjustment</h3>
              <div class="button-row" id="footPresetGroup">
                <button class="btn preset" data-angle="0">0°</button><button class="btn preset" data-angle="15">15°</button><button class="btn preset" data-angle="30">30°</button>
              </div>
              <div class="button-row"><button class="btn" id="footUpBtn">▲ Raise Foot</button><button class="btn" id="footDownBtn">▼ Lower Foot</button></div>
            </div>
          </div>
        </div>
        <div class="section-column">
          <div class="panel-card control-block">
            <h2>⚖️ Lateral Turn & Balance</h2>
            <div class="visual-viewport">
              <div class="view-container transverse-container"><div class="transverse-deck"><div class="deck-wing-l" id="leftHalf"></div><div class="deck-wing-r" id="rightHalf"></div><div class="pivot-pin"></div></div></div>
              <div class="telemetry-output">Transverse Balance <span id="latDisplay">Left 0° · Right 0°</span></div>
            </div>
            <div class="control-subgroup">
              <h3>Lateral Tilt Rotation</h3>
              <div class="button-row"><button class="btn" id="leftUpBtn">◀ Left Up</button><button class="btn" id="leftDownBtn">Left Down ▶</button></div>
              <div class="button-row"><button class="btn" id="rightUpBtn">▶ Right Up</button><button class="btn" id="rightDownBtn">◀ Right Down</button></div>
            </div>
          </div>
          <div class="panel-card">
            <div class="header-toggle-wrap">
              <h2>👁️ Eye-Blink Control</h2>
              <div class="switch-box"><span>Off</span><label class="switch"><input type="checkbox" id="blinkToggle" checked /><span class="slider"></span></label><span class="active-state">On</span></div>
            </div>
            <div id="blinkPanel" class="blink-terminal">
              <div style="display: flex; flex-direction: column; gap: 0.4rem; padding-top: 0.25rem; font-size: 0.7rem; color: var(--text-muted);">
                <div><strong>Short Blink:</strong> Hover</div><div><strong>Long Blink:</strong> Select</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
    <script>
      (() => {
        let headAngle = 0, footAngle = 0, leftTilt = 0, rightTilt = 0;
        const MAX_HEAD = 60, MAX_FOOT = 30, MAX_TILT = 25;
        let blinkActive = false;

        const headSeg = document.getElementById("headSeg"), footSegContainer = document.getElementById("footSegContainer"), footFlapSeg = document.getElementById("footFlapSeg"), leftHalf = document.getElementById("leftHalf"), rightHalf = document.getElementById("rightHalf"), sagDisplay = document.getElementById("sagDisplay"), latDisplay = document.getElementById("latDisplay"), headPresetBtns = document.querySelectorAll("#headPresetGroup .btn.preset"), footPresetBtns = document.querySelectorAll("#footPresetGroup .btn.preset"), blinkToggle = document.getElementById("blinkToggle"), blinkPanel = document.getElementById("blinkPanel");

        const statusBanner = document.getElementById("statusBanner");
        let bannerTimer = null;
        function showBanner(text, autoHideMs) {
          statusBanner.textContent = text;
          statusBanner.classList.add("active");
          if (bannerTimer) clearTimeout(bannerTimer);
          if (autoHideMs) bannerTimer = setTimeout(() => statusBanner.classList.remove("active"), autoHideMs);
        }
        function sendCmd(path) {
          fetch(path, { method: "GET" })
            .then(res => res.text())
            .then(text => {
              if (text.startsWith("BUSY") || text.startsWith("ESTOP")) {
                const match = text.match(/(\d+)\s*ms/);
                const secs = match ? Math.ceil(parseInt(match[1], 10) / 1000) : null;
                showBanner(secs ? `${text.split(":")[0]} - try again in ${secs}s` : text, (secs || 3) * 1000);
              } else {
                statusBanner.classList.remove("active");
              }
            })
            .catch(() => {});
        }
        function refreshUI() {
          headSeg.style.transform = `rotate(${headAngle}deg)`; footSegContainer.style.transform = `rotate(${-footAngle}deg)`; footFlapSeg.style.transform = `rotate(${footAngle * 1.5}deg)`;
          sagDisplay.textContent = `Head ${headAngle}° · Foot ${footAngle}°`; leftHalf.style.transform = `rotate(${leftTilt}deg)`; rightHalf.style.transform = `rotate(${-rightTilt}deg)`;
          latDisplay.textContent = `Left ${leftTilt}° · Right ${rightTilt}°`;
          headPresetBtns.forEach(btn => btn.classList.toggle("active", headAngle === parseInt(btn.dataset.angle, 10)));
          footPresetBtns.forEach(btn => btn.classList.toggle("active", footAngle === parseInt(btn.dataset.angle, 10)));
        }
        function setHeadPreset(angle) { removeDragClasses(); headAngle = angle; sendCmd(`/api?head=${headAngle}`); refreshUI(); }
        function setFootPreset(angle) { removeDragClasses(); footAngle = angle; sendCmd(`/api?foot=${footAngle}`); refreshUI(); }
        function moveSagittal(part, dir) {
          removeDragClasses();
          if (part === "head") { headAngle = dir === "up" ? Math.min(MAX_HEAD, headAngle + 5) : Math.max(0, headAngle - 5); sendCmd(`/api?head=${headAngle}`); }
          else { footAngle = dir === "up" ? Math.min(MAX_FOOT, footAngle + 5) : Math.max(0, footAngle - 5); sendCmd(`/api?foot=${footAngle}`); }
          refreshUI();
        }
        function moveLateral(side, dir) {
          removeDragClasses();
          if (side === "left") { if (rightTilt > 0) rightTilt = 0; leftTilt = dir === "up" ? Math.min(MAX_TILT, leftTilt + 5) : Math.max(0, leftTilt - 5); sendCmd(`/api?leftTilt=${leftTilt}`); }
          else { if (leftTilt > 0) leftTilt = 0; rightTilt = dir === "up" ? Math.min(MAX_TILT, rightTilt + 5) : Math.max(0, rightTilt - 5); sendCmd(`/api?rightTilt=${rightTilt}`); }
          refreshUI();
        }
        function removeDragClasses() { [headSeg, footSegContainer, footFlapSeg, leftHalf, rightHalf].forEach(el => el.classList.remove("dragging-active")); }
        function addDragClasses() { [headSeg, footSegContainer, footFlapSeg, leftHalf, rightHalf].forEach(el => el.classList.add("dragging-active")); }
        function toggleBlink(enable) { blinkActive = enable; sendCmd(`/api?blinkMode=${enable ? 1 : 0}`); blinkPanel.classList.toggle("active", enable); }

        document.getElementById("headUpBtn").addEventListener("click", () => moveSagittal("head", "up"));
        document.getElementById("headDownBtn").addEventListener("click", () => moveSagittal("head", "down"));
        document.getElementById("footUpBtn").addEventListener("click", () => moveSagittal("foot", "up"));
        document.getElementById("footDownBtn").addEventListener("click", () => moveSagittal("foot", "down"));
        document.getElementById("leftUpBtn").addEventListener("click", () => moveLateral("left", "up"));
        document.getElementById("leftDownBtn").addEventListener("click", () => moveLateral("left", "down"));
        document.getElementById("rightUpBtn").addEventListener("click", () => moveLateral("right", "up"));
        document.getElementById("rightDownBtn").addEventListener("click", () => moveLateral("right", "down"));
        headPresetBtns.forEach(btn => btn.addEventListener("click", () => setHeadPreset(parseInt(btn.dataset.angle, 10))));
        footPresetBtns.forEach(btn => btn.addEventListener("click", () => setFootPreset(parseInt(btn.dataset.angle, 10))));
        blinkToggle.addEventListener("change", (e) => toggleBlink(e.target.checked));

        let isDragging = false, activePart = null, startY = 0, startAngle = 0;
        function getClientY(e) { return e.touches ? e.touches[0].clientY : e.clientY; }
        function handleDragStart(part, initialAngle, e) { isDragging = true; activePart = part; startY = getClientY(e); startAngle = initialAngle; addDragClasses(); }

        headSeg.addEventListener("mousedown", (e) => handleDragStart("head", headAngle, e));
        headSeg.addEventListener("touchstart", (e) => handleDragStart("head", headAngle, e));
        footSegContainer.addEventListener("mousedown", (e) => handleDragStart("foot", footAngle, e));
        footSegContainer.addEventListener("touchstart", (e) => handleDragStart("foot", footAngle, e));
        leftHalf.addEventListener("mousedown", (e) => handleDragStart("left", leftTilt, e));
        leftHalf.addEventListener("touchstart", (e) => handleDragStart("left", leftTilt, e));
        rightHalf.addEventListener("mousedown", (e) => handleDragStart("right", rightTilt, e));
        rightHalf.addEventListener("touchstart", (e) => handleDragStart("right", rightTilt, e));

        window.addEventListener("mousemove", handleDragMove);
        window.addEventListener("touchmove", handleDragMove, { passive: false });
        window.addEventListener("mouseup", handleDragEnd);
        window.addEventListener("touchend", handleDragEnd);

        function handleDragMove(e) {
          if (!isDragging) return; if (e.touches) e.preventDefault();
          const currentY = getClientY(e); const deltaY = startY - currentY; const sensitivity = 1.5; let computedAngle = startAngle + deltaY / sensitivity;
          if (activePart === "head") { headAngle = Math.max(0, Math.min(MAX_HEAD, Math.round(computedAngle))); sendCmd(`/api?head=${headAngle}`); }
          else if (activePart === "foot") { footAngle = Math.max(0, Math.min(MAX_FOOT, Math.round(computedAngle))); sendCmd(`/api?foot=${footAngle}`); }
          else if (activePart === "left") { if (rightTilt > 0) rightTilt = 0; leftTilt = Math.max(0, Math.min(MAX_TILT, Math.round(computedAngle))); sendCmd(`/api?leftTilt=${leftTilt}`); }
          else if (activePart === "right") { if (leftTilt > 0) leftTilt = 0; rightTilt = Math.max(0, Math.min(MAX_TILT, Math.round(computedAngle))); sendCmd(`/api?rightTilt=${rightTilt}`); }
          refreshUI();
        }
        function handleDragEnd() { if (isDragging) { isDragging = false; activePart = null; setTimeout(removeDragClasses, 50); } }
        removeDragClasses(); headAngle = 0; footAngle = 0; refreshUI();
        toggleBlink(true);
      })();
    </script>
  </body>
</html>
)=====";

void handleRoot() {
  server.send(200, "text/html", html_page);
}

// PRIORITY 3 (lowest): dropped whenever remote OR Nextion outranks it.
void handleAPI() {
  // Mode toggle - always allowed, it only changes which input method is
  // active, it never commands a movement, so it doesn't need arbitration.
  if (server.hasArg("blinkMode")) {
    blinkModeEnabled = server.arg("blinkMode").toInt() == 1;
    applyTouchModeForCurrentPage();
    server.send(200, "text/plain", blinkModeEnabled ? "BLINK_MODE_ON" : "BLINK_MODE_OFF");
    return;
  }

  if (emergencyStopActive) {
    Serial.println("[BLOCKED] Web command ignored - E-STOP active");
    server.send(200, "text/plain", "ESTOP: system halted, release kill switch first");
    return;
  }

  if (remoteHasPriority()) {
    unsigned long remaining = REMOTE_PRIORITY_COOLDOWN_MS - (millis() - lastRemoteActivityMillis);
    // headManualJogActive / isHomingActive have no fixed end time, so just
    // report a nominal window in that case rather than a bogus countdown.
    if (headManualJogActive || isHomingActive) remaining = REMOTE_PRIORITY_COOLDOWN_MS;
    Serial.print("[BLOCKED] Web command ignored - remote has priority for ");
    Serial.print(remaining);
    Serial.println(" ms");
    server.send(200, "text/plain", "BUSY: remote control active, try again in " + String(remaining) + " ms");
    return;
  }

  if (nextionHasPriority()) {
    unsigned long remaining = NEXTION_PRIORITY_COOLDOWN_MS - (millis() - lastNextionActivityMillis);
    Serial.print("[BLOCKED] Web command ignored - Nextion has priority for another ");
    Serial.print(remaining);
    Serial.println(" ms");
    server.send(200, "text/plain", "BUSY: Nextion active, try again in " + String(remaining) + " ms");
    return;
  }

  bool updated = false;
  bool motorCommand = false;
  float requestedRoll = 0.0;

  if (server.hasArg("head")) {
    headAngle = server.arg("head").toInt();
    updated = true;
    motorCommand = true;
    requestedRoll = headAngle;
  }
  if (server.hasArg("foot")) {
    footAngle = server.arg("foot").toInt();
    updated = true;
  }
  if (server.hasArg("leftTilt")) {
    leftTilt = server.arg("leftTilt").toInt();
    if (leftTilt > 0) rightTilt = 0;
    updated = true;
  }
  if (server.hasArg("rightTilt")) {
    rightTilt = server.arg("rightTilt").toInt();
    if (rightTilt > 0) leftTilt = 0;
    updated = true;
  }

  if (updated) {
    Serial.println("--- BED STATE UPDATE (WEB) ---");
    Serial.printf("Head Angle : %d deg\n", headAngle);
    Serial.printf("Foot Angle : %d deg\n", footAngle);
    Serial.printf("Left Tilt  : %d deg\n", leftTilt);
    Serial.printf("Right Tilt : %d deg\n", rightTilt);
    Serial.println("------------------------------\n");
  }

  if (motorCommand) {
    Serial.println("[SOURCE: WEB]");
    setBedTarget(requestedRoll);
  }

  server.send(200, "text/plain", "OK");
}

// ===================================================================
// MPU6050 (IMU) - complementary filter + fixed mount-offset correction
// ===================================================================
#define MPU6050_ADDR 0x68
int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

const float ACC_SCALE  = 16384.0;
const float GYRO_SCALE = 131.0;

// Fixed mechanical mounting correction (see original comment in the
// remote sketch for calibration procedure). Applied everywhere roll is
// computed, so it stays correct even if the bed boots while not flat.
const float IMU_ROLL_MOUNT_OFFSET_DEG = -1.2;

float accOffsetX = 0, accOffsetY = 0, accOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

float roll = 0;
float pitch = 0;
unsigned long lastMicros = 0;

class ComplementaryFilter {
  public:
    float alpha = 0.98;
    float angle = 0.0;
    float update(float gyroRate, float accAngle, float dt) {
      float gyroAngle = angle + gyroRate * dt;
      angle = alpha * gyroAngle + (1.0 - alpha) * accAngle;
      return angle;
    }
};

ComplementaryFilter cFilterRoll;
ComplementaryFilter cFilterPitch;

// ===================================================================
// AS5600 ENCODER (with light noise mitigation - see header note re: WiFi AP)
// ===================================================================
const unsigned long ENCODER_SAMPLE_INTERVAL_MS = 4;
unsigned long lastEncoderMillis = 0;

volatile long rotations = 0;
float lastAngle = 0;

// 3-sample median of analogRead to knock down single-sample ADC spikes
// (WiFi radio activity is a known noise source on this pin). This does
// NOT fix the underlying issue, it just makes single-sample glitches
// less likely to register as a full wraparound event below.
int readEncoderRawMedian() {
  int a = analogRead(ENCODER_PIN);
  int b = analogRead(ENCODER_PIN);
  int c = analogRead(ENCODER_PIN);
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b; // median
}

void readEncoder() {
  int raw = readEncoderRawMedian();
  float angle = (raw / 4095.0) * 360.0;
  float diff = angle - lastAngle;

  if (diff > 200.0) {
    rotations++;
  }
  if (diff < -200.0) {
    rotations--;
    if (rotations < 0) rotations = 0;
  }
  lastAngle = angle;
}

// ===================================================================
// TORSO/HEAD MOTOR (Cytron MD30C) - shared by remote/Nextion/web
// Direction convention: DIR_PIN=LOW is "increasing" (see header note #1)
// ===================================================================
const int MOTOR_SPEED = 255;
long targetRotations = 0;
bool motorMoving = false;

// Extra encoder "rotations" of motor-shaft travel needed to take up
// mechanical slack in the torso leadscrew whenever the motor reverses
// direction. Tune on the bench per the original comment: jog up/down/up
// and count rotations before the bed visibly moves again after a reversal.
const long BACKLASH_COMPENSATION = 6;

// Inside this many rotations of target, motor drops to MOTOR_SPEED_SLOW
// to avoid overshoot between 4ms encoder polls.
const long HOMING_SLOWDOWN_ZONE = 15;
const int  MOTOR_SPEED_SLOW     = 90;

// Last direction the torso motor actually drove: +1 increasing, -1
// decreasing, 0 unknown. Persists across stops (including manual jog and
// Nextion/web commanded stops) so the next move - regardless of source -
// knows whether it needs to pay the backlash toll.
int8_t torsoLastDirection = 0;

void motorStop() { analogWrite(MOTOR_PWM_PIN, 0); }
void motorDriveIncreasing(int spd = MOTOR_SPEED) { digitalWrite(MOTOR_DIR_PIN, LOW); analogWrite(MOTOR_PWM_PIN, spd); }
void motorDriveDecreasing(int spd = MOTOR_SPEED) { digitalWrite(MOTOR_DIR_PIN, HIGH); analogWrite(MOTOR_PWM_PIN, spd); }

void setupMotor() {
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  motorStop();
}

long applyBacklashCompensation(long rawTarget) {
  int8_t neededDir = (rawTarget > rotations) ? 1 : (rawTarget < rotations ? -1 : 0);

  if (neededDir != 0 && torsoLastDirection != 0 && neededDir != torsoLastDirection) {
    rawTarget += (long)neededDir * BACKLASH_COMPENSATION;
    Serial.print("[BACKLASH COMP] Direction reversal -> adding ");
    Serial.print(BACKLASH_COMPENSATION);
    Serial.println(" rotations of slack take-up to target.");
  }

  // rotations is hard-clamped at 0 in readEncoder(), so a compensated
  // target below 0 could never actually be reached.
  if (rawTarget < 0) rawTarget = 0;

  return rawTarget;
}

// ===================================================================
// LEG/FOOT MOTOR - remote-only, open-loop, digital bypass (no PWM)
// ===================================================================
void legMotorStop()      { digitalWrite(LEG_MOTOR_PWM_PIN, LOW); }
void legMotorDriveUp()   { digitalWrite(LEG_MOTOR_DIR_PIN, LOW); digitalWrite(LEG_MOTOR_PWM_PIN, HIGH); }
void legMotorDriveDown() { digitalWrite(LEG_MOTOR_DIR_PIN, HIGH);  digitalWrite(LEG_MOTOR_PWM_PIN, HIGH); }

void setupLegMotor() {
  pinMode(LEG_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEG_MOTOR_DIR_PIN, OUTPUT);
  legMotorStop();
}

void commandLegManual(int dir) {
  if (dir == lastLegState) return;
  lastLegState = dir;
  if (dir != 0) markRemoteActive();

  if (emergencyStopActive) { legMotorStop(); return; }

  if (dir == 1) { Serial.println("CMD: LEG RAISING (real motor, open-loop)..."); legMotorDriveUp(); }
  else if (dir == -1) { Serial.println("CMD: LEG LOWERING (real motor, open-loop)..."); legMotorDriveDown(); }
  else { Serial.println("CMD: LEG STOPPED."); legMotorStop(); }
}

void commandHeadManual(int dir) {
  if (dir == lastHeadState) return;
  lastHeadState = dir;
  if (dir != 0) markRemoteActive();

  if (emergencyStopActive) { motorStop(); headManualJogActive = false; return; }

  if (dir == 1) {
    Serial.println("CMD: HEAD/TORSO LIFTING (real motor, manual jog - REMOTE)...");
    headManualJogActive = true;
    motorMoving = false;
    motorDriveIncreasing();
    torsoLastDirection = 1;
  } else if (dir == -1) {
    Serial.println("CMD: HEAD/TORSO LOWERING (real motor, manual jog - REMOTE)...");
    headManualJogActive = true;
    motorMoving = false;
    motorDriveDecreasing();
    torsoLastDirection = -1;
  } else {
    Serial.println("CMD: HEAD/TORSO STOPPED (remote release).");
    motorStop();
    headManualJogActive = false;
    targetRotations = rotations; // discard any stale Nextion/web target
    markRemoteActive(); // start the priority-hold window on release too
  }
}

void commandTiltSim(int dir) {
  if (dir == lastTiltState) return;
  lastTiltState = dir;
  if (dir != 0) markRemoteActive();

  if (dir == 1) Serial.println("CMD: SIDE TILT -> RAISING LEFT [SIMULATED - no tilt motor installed]");
  else if (dir == 2) Serial.println("CMD: SIDE TILT -> RAISING RIGHT (pushing Left to 0) [SIMULATED]");
  else Serial.println("CMD: SIDE TILT STOPPED. [SIMULATED]");
}

void haltAllMotorsHard() {
  motorStop();
  motorMoving = false;
  headManualJogActive = false;
  targetRotations = rotations;
  legMotorStop();
  lastHeadState = 0;
  lastLegState = 0;
  commandTiltSim(0);
  // torsoLastDirection intentionally NOT reset - mechanical slack state
  // survives a hard stop, next move still needs to know which way it
  // last actually drove.
}

// ===================================================================
// REMOTE: HOMING SEQUENCE (physical, closed-loop on stage 3)
// ===================================================================
void updateHomingSimulation() {
  if (!isHomingActive) return;
  markRemoteActive();

  if (emergencyStopActive) {
    Serial.println("[ HOMING ABORTED ] Kill switch engaged.");
    isHomingActive = false;
    homingStage = 0;
    return;
  }

  unsigned long currentMillis = millis();

  if (homingStage == 1) {
    if (currentMillis - homingTimer > 2000) {
      Serial.println("HOMING Step 1 Complete [SIMULATED]. Now zeroing leg axis...");
      homingStage = 2;
      homingTimer = currentMillis;
    }
  } else if (homingStage == 2) {
    if (currentMillis - homingTimer > 2000) {
      Serial.println("HOMING Step 2 Complete [SIMULATED]. Activating physical closed-loop homing for Head/Torso (Target: 0 deg)...");
      long rawTarget = rollToRotations(0.0);
      targetRotations = applyBacklashCompensation(rawTarget);
      motorMoving = true;
      headManualJogActive = false;
      homingStage = 3;
      homingTimer = currentMillis;
    }
  } else if (homingStage == 3) {
    if (!motorMoving) {
      Serial.println("[ HOMING COMPLETE ] Head/Torso zeroed via closed-loop encoder tracking. System safe.");
      isHomingActive = false;
      homingStage = 0;
    }
  }
}

// ===================================================================
// REMOTE: 74HC165 shift register polling (PRIORITY 1)
// ===================================================================
void updateRemoteLED(bool isCommandActive) {
  if (isCommandActive) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastRemoteBlinkTime >= 100) {
      lastRemoteBlinkTime = currentMillis;
      remoteLedState = !remoteLedState;
      digitalWrite(LED_BUILTIN, remoteLedState);
    }
  } else if (remoteLedState != HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);
    remoteLedState = HIGH;
  }
}

byte read74HC165() {
  byte value = 0;
  for (int i = 0; i < 8; ++i) {
    int bitValue = digitalRead(REMOTE_DATA_PIN);
    value |= (bitValue << (7 - i));
    digitalWrite(REMOTE_CLOCK_PIN, HIGH);
    delayMicroseconds(5); // 5us for 1-meter cable stability
    digitalWrite(REMOTE_CLOCK_PIN, LOW);
  }
  return value;
}

void pollRemote() {
  static unsigned long lastRemotePoll = 0;
  unsigned long now = millis();
  if (now - lastRemotePoll < 15) return;
  lastRemotePoll = now;

  digitalWrite(REMOTE_LATCH_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(REMOTE_LATCH_PIN, HIGH);
  byte state = read74HC165();

  bool btnHoming    = bitRead(state, 0);
  bool btnRightUp   = bitRead(state, 1);
  bool btnLegUp     = bitRead(state, 2);
  bool btnHeadUp    = bitRead(state, 3);
  bool btnHeadDown  = bitRead(state, 4);
  bool btnLegDown   = bitRead(state, 5);
  bool btnLeftUp    = bitRead(state, 6);
  bool btnKill      = bitRead(state, 7);

  bool isMoving = (btnHeadUp || btnHeadDown || btnLegUp || btnLegDown ||
                    btnLeftUp || btnRightUp || isHomingActive);

  if (isMoving) markRemoteActive();

  // --- PRIORITY 1a: GLOBAL KILL SWITCH (outranks everything, incl. Nextion/web) ---
  if (btnKill) {
    if (!lastKillState) {
      Serial.println("\n[ !!! EMERGENCY KILL SWITCH ACTIVATED !!! ]");
      lastKillState = true;
    }
    emergencyStopActive = true;
    isHomingActive = false;
    homingStage = 0;
    haltAllMotorsHard();
    markRemoteActive();
    digitalWrite(LED_BUILTIN, LOW);
    return;
  } else if (lastKillState) {
    Serial.println("[ KILL SWITCH RELEASED - System Ready ]");
    lastKillState = false;
    emergencyStopActive = false;
    markRemoteActive(); // hold priority briefly after release
    digitalWrite(LED_BUILTIN, HIGH);
  }

  updateRemoteLED(isMoving);

  // --- PRIORITY 1b: HOMING SEQUENCE ---
  if (btnHoming && !isHomingActive) {
    Serial.println("\n[ HOMING SEQUENCE INITIATED ] Executing multi-axis systemic reset...");
    isHomingActive = true;
    homingStage = 1;
    homingTimer = millis();
    haltAllMotorsHard();
  }

  if (isHomingActive) {
    updateHomingSimulation();
    return; // hard lock manual inputs during homing
  }

  // --- PRIORITY 1c: MANUAL AXIS JOG ---
  if (btnHeadUp) commandHeadManual(1);
  else if (btnHeadDown) commandHeadManual(-1);
  else commandHeadManual(0);

  if (btnLegUp) commandLegManual(1);
  else if (btnLegDown) commandLegManual(-1);
  else commandLegManual(0);

  if (btnLeftUp) commandTiltSim(1);
  else if (btnRightUp) commandTiltSim(2);
  else commandTiltSim(0);
}

// ===================================================================
// CALIBRATION LOOKUP (shared by remote homing, Nextion, and web)
// ===================================================================
long rollToRotations(float targetRoll) {
  float rollFirst = pgm_read_float(&calRoll[0]);
  float rollLast  = pgm_read_float(&calRoll[CAL_TABLE_SIZE - 1]);
  bool increasing = (rollLast >= rollFirst);

  if (increasing) {
    if (targetRoll <= rollFirst) return pgm_read_dword(&calRotations[0]);
    if (targetRoll >= rollLast)  return pgm_read_dword(&calRotations[CAL_TABLE_SIZE - 1]);
  } else {
    if (targetRoll >= rollFirst) return pgm_read_dword(&calRotations[0]);
    if (targetRoll <= rollLast)  return pgm_read_dword(&calRotations[CAL_TABLE_SIZE - 1]);
  }

  for (int i = 0; i < CAL_TABLE_SIZE - 1; i++) {
    float roll0 = pgm_read_float(&calRoll[i]);
    float roll1 = pgm_read_float(&calRoll[i + 1]);
    bool inRange = increasing ? (targetRoll >= roll0 && targetRoll <= roll1) : (targetRoll <= roll0 && targetRoll >= roll1);

    if (inRange) {
      int32_t rot0 = pgm_read_dword(&calRotations[i]);
      int32_t rot1 = pgm_read_dword(&calRotations[i + 1]);
      float t = (roll1 == roll0) ? 0.0f : (targetRoll - roll0) / (roll1 - roll0);
      return rot0 + (long)lround(t * (rot1 - rot0));
    }
  }
  return pgm_read_dword(&calRotations[CAL_TABLE_SIZE - 1]);
}

// ===================================================================
// SHARED CLOSED-LOOP MOTOR CONTROL (backlash-aware + creep speed).
// Services whichever target was last legitimately set - by remote
// homing, Nextion, or web - since all three now go through the same
// applyBacklashCompensation() path via setBedTarget() / homing stage 2.
// ===================================================================
void updateMotorControl() {
#if SIMULATE_MOTOR
  return;
#endif
  if (emergencyStopActive) { motorStop(); motorMoving = false; return; }
  if (headManualJogActive) return; // remote manual jog owns the motor directly
  if (!motorMoving) return;

  long error = targetRotations - rotations;

  if (error == 0) {
    motorStop();
    motorMoving = false;
    Serial.print("\n[SUCCESS] Position reached. Rotations: "); Serial.print(rotations);
    Serial.print(" | Active Roll: "); Serial.println(roll, 2);
    return;
  }

  int speed = (labs(error) <= HOMING_SLOWDOWN_ZONE) ? MOTOR_SPEED_SLOW : MOTOR_SPEED;

  if (error > 0) {
    motorDriveIncreasing(speed);
    torsoLastDirection = 1;
  } else {
    motorDriveDecreasing(speed);
    torsoLastDirection = -1;
  }
}

// Entry point used by BOTH Nextion (interpretBedCommand) and web
// (handleAPI) to request a torso angle. Callers are responsible for
// doing their own priority checks BEFORE calling this - this function
// just converts angle -> backlash-compensated target and arms the motor.
void setBedTarget(float targetAngle) {
#if SIMULATE_MOTOR
  targetRotations = applyBacklashCompensation(rollToRotations(targetAngle));
  motorMoving = false;
  Serial.print("\n[SIMULATED MOTOR] Would move to angle: "); Serial.print(targetAngle, 2);
  Serial.print(" deg -> target rotations: "); Serial.println(targetRotations);
  return;
#endif
  long rawTarget = rollToRotations(targetAngle);
  targetRotations = applyBacklashCompensation(rawTarget);
  motorMoving = true;
  Serial.print("\n>>> New target angle: "); Serial.print(targetAngle, 2);
  Serial.print(" deg -> Target rotations: "); Serial.println(targetRotations);
}

// ===================================================================
// NEXTION NAVIGATION / TOUCH LOGIC (PRIORITY 2)
// ===================================================================
int hoverColor = 1055;
int defaultColor = 65535;

int currentPage = 0;
int currentIndex = 0;

String page0_btns[] = {"b1", "b6", "b2", "b0"};
String page1_btns[] = {"b1", "b2", "b3", "b4", "b0"};
String page2_btns[] = {"b9", "b10", "b0"};

const int page0_size = 4;
const int page1_size = 5;
const int page2_size = 3;

String getCurrentButtonID() {
  if (currentPage == 0) return page0_btns[currentIndex];
  if (currentPage == 1) return page1_btns[currentIndex];
  if (currentPage == 2) return page2_btns[currentIndex];
  return "";
}

void endNextionCmd() {
  Serial2.write(0xFF); Serial2.write(0xFF); Serial2.write(0xFF);
}

void highlightCurrentButton(int colorCode) {
  String target = getCurrentButtonID();
  Serial2.print(target + ".bco=" + String(colorCode));
  endNextionCmd();
  Serial2.print("ref " + target);
  endNextionCmd();
}

void clearAllButtonsOnPage() {
  if (currentPage == 0) {
    for (int i = 0; i < page0_size; i++) { Serial2.print(page0_btns[i] + ".bco=" + String(defaultColor)); endNextionCmd(); }
  } else if (currentPage == 1) {
    for (int i = 0; i < page1_size; i++) { Serial2.print(page1_btns[i] + ".bco=" + String(defaultColor)); endNextionCmd(); }
  } else if (currentPage == 2) {
    for (int i = 0; i < page2_size; i++) { Serial2.print(page2_btns[i] + ".bco=" + String(defaultColor)); endNextionCmd(); }
  }
}

void applyTouchModeForCurrentPage() {
  bool touchEnabled = !blinkModeEnabled;
  String* btns;
  int size;
  if (currentPage == 0)      { btns = page0_btns; size = page0_size; }
  else if (currentPage == 1) { btns = page1_btns; size = page1_size; }
  else                       { btns = page2_btns; size = page2_size; }

  for (int i = 0; i < size; i++) {
    Serial2.print("tsw " + btns[i] + "," + String(touchEnabled ? 1 : 0));
    endNextionCmd();
  }
  Serial.print("[MODE] Nextion physical touch ");
  Serial.println(touchEnabled ? "ENABLED" : "LOCKED (blink mode active)");
}

void handleNext() {
  highlightCurrentButton(defaultColor);
  currentIndex++;

  if (currentPage == 0 && currentIndex >= page0_size) currentIndex = 0;
  if (currentPage == 1 && currentIndex >= page1_size) currentIndex = 0;
  if (currentPage == 2 && currentIndex >= page2_size) currentIndex = 0;

  highlightCurrentButton(hoverColor);
}

void handleSelect() {
  String target = getCurrentButtonID();
  highlightCurrentButton(defaultColor);

  Serial2.print("click " + target + ",1"); endNextionCmd();
  delay(100);
  Serial2.print("click " + target + ",0"); endNextionCmd();

  bool pageChanged = false;
  int nextExecutionPage = currentPage;

  if (currentPage == 0) {
    if (target == "b1" || target == "b2") { nextExecutionPage = 1; pageChanged = true; }
    else if (target == "b0" || target == "b6") { nextExecutionPage = 2; pageChanged = true; }
  } else if (currentPage == 1 || currentPage == 2) {
    if (target == "b0") { nextExecutionPage = 0; pageChanged = true; }
  }

  if (pageChanged) {
    Serial2.print("page page" + String(nextExecutionPage));
    endNextionCmd();
    delay(250);
    currentPage = nextExecutionPage;
    currentIndex = 0;
    clearAllButtonsOnPage();
    highlightCurrentButton(hoverColor);
    applyTouchModeForCurrentPage();
  } else {
    highlightCurrentButton(hoverColor);
  }
}

// Actual bed-angle requests arriving from the Nextion screen (either a
// direct physical touch or a blink-triggered click on an angle button)
// funnel through here. This is the single choke point where remote
// priority is enforced against Nextion.
void interpretBedCommand(String cmd) {
  char foundCommand = ' ';
  for (int i = 0; i < cmd.length() - 1; i++) {
    if (cmd.charAt(i) == 'H') {
      foundCommand = cmd.charAt(i + 1);
      break;
    }
  }

  if (foundCommand == '0' || foundCommand == '1' || foundCommand == '2' || foundCommand == '3') {
    if (emergencyStopActive) {
      Serial.println("[BLOCKED] Nextion command ignored - E-STOP active");
      return;
    }
    if (remoteHasPriority()) {
      Serial.println("[BLOCKED] Nextion command ignored - remote has priority");
      return;
    }

    markNextionActive();
    Serial.println("[SOURCE: NEXTION]");
    if (foundCommand == '0') setBedTarget(0.0);
    else if (foundCommand == '1') setBedTarget(15.0);
    else if (foundCommand == '2') setBedTarget(30.0);
    else if (foundCommand == '3') setBedTarget(60.0);
  } else {
    Serial.print("Command acknowledged. Length: ");
    Serial.println(cmd.length());
  }
}

// ===================================================================
// DEBUG STATUS (combined)
// ===================================================================
void printDebugStatus() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastDebugPrintMillis < DEBUG_PRINT_INTERVAL_MS) return;
  lastDebugPrintMillis = currentMillis;

  const char* source = "IDLE";
  if (emergencyStopActive) source = "ESTOP";
  else if (headManualJogActive) source = "REMOTE-JOG";
  else if (isHomingActive) source = "REMOTE-HOMING";
  else if (remoteHasPriority()) source = "REMOTE-HOLD";
  else if (nextionHasPriority()) source = "NEXTION";
  else if (motorMoving) source = "WEB/AUTO";

  Serial.print("[STATUS] Src: "); Serial.print(source);
  Serial.print(" | Rotations: "); Serial.print(rotations);
  Serial.print(" | Target: "); Serial.print(targetRotations);
  Serial.print(" | Roll: "); Serial.println(roll, 2);
}

// ===================================================================
// SENSOR SETUP HELPERS
// ===================================================================
void setupMPU() {
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
}

void readMPU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU6050_ADDR, (uint8_t)14, true);

  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  GyX = (Wire.read() << 8) | Wire.read();
  GyY = (Wire.read() << 8) | Wire.read();
  GyZ = (Wire.read() << 8) | Wire.read();
}

void calibrateMPU() {
  Serial.println("Calibrating MPU6050 Gyro...");
  long gx = 0, gy = 0, gz = 0;
  unsigned long startTime = millis();
  int samples = 0;

  while (millis() - startTime < 3000) {
    readMPU();
    gx += GyX; gy += GyY; gz += GyZ;
    samples++;
    delay(5);
  }

  gyroOffsetX = (float)gx / samples;
  gyroOffsetY = (float)gy / samples;
  gyroOffsetZ = (float)gz / samples;
}

// ===================================================================
// SETUP
// ===================================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  pinMode(16, INPUT_PULLUP);
  Serial2.setTimeout(10);
  Serial.println("\nInitializing MediTilt Pro (Remote > Nextion > Web, fully merged)...");
  

#if SIMULATE_MOTOR
  Serial.println("[SIMULATION] Skipping MPU6050 init/calibration - no IMU connected");
  rotations = rollToRotations(0.0);
#else
  Wire.begin(4, 5);
  setupMPU();
  delay(200);
  calibrateMPU();

  readMPU();
  float ax = AcX - accOffsetX;
  float ay = AcY - accOffsetY;
  float az = AcZ - accOffsetZ;
  cFilterRoll.angle  = (atan2(ay, az) * 180.0 / PI) - IMU_ROLL_MOUNT_OFFSET_DEG;
  cFilterPitch.angle = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  lastMicros = micros();

  rotations = rollToRotations(cFilterRoll.angle);
#endif

  setupMotor();
  setupLegMotor();

  // Remote shift register
  pinMode(REMOTE_LATCH_PIN, OUTPUT);
  pinMode(REMOTE_CLOCK_PIN, OUTPUT);
  digitalWrite(REMOTE_LATCH_PIN, HIGH);
  pinMode(REMOTE_DATA_PIN, INPUT_PULLDOWN); // defaults to 0 if tether breaks
  digitalWrite(REMOTE_CLOCK_PIN, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  int raw = analogRead(ENCODER_PIN);
  lastAngle = (raw / 4095.0) * 360.0;

  delay(500);
  clearAllButtonsOnPage();
  highlightCurrentButton(hoverColor);
  applyTouchModeForCurrentPage();

  // WiFi AP + web server (see header note #2 re: GPIO36 noise)
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started!");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/api", handleAPI);
  server.begin();
  Serial.println("HTTP Web Server Started, ready for connections.");

  Serial.println("Physical remote subsystem online and armed (top priority).");
}

// ===================================================================
// LOOP
// ===================================================================
void loop() {
  unsigned long currentMillis = millis();

  // Unconditional encoder polling
  if (currentMillis - lastEncoderMillis >= ENCODER_SAMPLE_INTERVAL_MS) {
    lastEncoderMillis = currentMillis;
    readEncoder();
  }

  // PRIORITY 1: hardware remote (kill switch / homing / manual jog)
  pollRemote();

  // PRIORITY 2: Nextion - USB blink navigation bytes
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    Serial.print("[USB RX] Got byte: "); Serial.println(incomingChar);
    if (!blinkModeEnabled) {
      Serial.println("[IGNORED] Blink mode is off - Nextion touch mode is active");
    } else if (remoteHasPriority()) {
      Serial.println("[BLOCKED] Blink nav ignored - remote has priority");
    } else {
      markNextionActive();
      if (incomingChar == '0') {
        handleNext();
        Serial.println("ACK:0");
      } else if (incomingChar == '1') {
        handleSelect();
        Serial.println("ACK:1");
      }
    }
  }

  // PRIORITY 2: Nextion - physical screen presses (H-commands over Serial2)
  if (Serial2.available() > 0) {
    String incomingCmd = Serial2.readStringUntil('\n');
    incomingCmd.trim();
    if (incomingCmd.length() > 0) {
      interpretBedCommand(incomingCmd); // internally checks remoteHasPriority()
    }
  }

  // IMU update (20ms interval)
#if !SIMULATE_MOTOR
  static unsigned long lastSampleMillis = 0;
  if (currentMillis - lastSampleMillis >= 20) {
    lastSampleMillis = currentMillis;

    readMPU();
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0;
    lastMicros = now;

    float ax = AcX - accOffsetX; float ay = AcY - accOffsetY; float az = AcZ - accOffsetZ;
    float gxRate = (GyX - gyroOffsetX) / GYRO_SCALE;
    float gyRate = (GyY - gyroOffsetY) / GYRO_SCALE;

    float rollAcc  = (atan2(ay, az) * 180.0 / PI) - IMU_ROLL_MOUNT_OFFSET_DEG;
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    roll  = cFilterRoll.update(gxRate, rollAcc, dt);
    pitch = cFilterPitch.update(gyRate, pitchAcc, dt);
  }
#endif

  // Shared closed-loop motor service (drives whatever legitimate target
  // is currently armed - remote homing, Nextion, or web - with backlash
  // compensation and creep-speed baked in)
  updateMotorControl();

  printDebugStatus();

  // PRIORITY 3: web dashboard (gated internally inside handleAPI())
  server.handleClient();
}