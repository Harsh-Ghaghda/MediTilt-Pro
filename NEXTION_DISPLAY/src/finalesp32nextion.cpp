#include <Arduino.h>
#include <Wire.h>
// #include "calibration_table.h" // Uncomment when you add this file to your 'include' folder

//========================================================================
// 1. UI COLOR SETTINGS & NAVIGATION TRACKING
//========================================================================
int hoverColor = 1055;     // Custom Blue highlight
int defaultColor = 65535;  // Default normal button color

int currentPage = 0;   
int currentIndex = 0;  

// Exact button IDs from your Nextion Layout
String page0_btns[] = {"b1", "b6", "b2", "b0"}; 
String page1_btns[] = {"b1", "b2", "b3", "b4", "b0"}; 
String page2_btns[] = {"b9", "b10", "b0"}; 

const int page0_size = 4;
const int page1_size = 5;
const int page2_size = 3;

//========================================================================
// 2. LOCKOUT TIMER VARIABLES (PHYSICAL REMOTE)
//========================================================================
unsigned long lockoutStartTime = 0;
bool isScreenLocked = false;
const unsigned long lockoutDuration = 5000; // 5 seconds

//========================================================================
// 3. MPU6050 (IMU) CONFIGURATION
//========================================================================
#define MPU6050_ADDR 0x68
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
const float ACC_SCALE  = 16384.0;
const float GYRO_SCALE = 131.0;
float accOffsetX = 0, accOffsetY = 0, accOffsetZ = 0;
unsigned long lastMicros = 0;

//========================================================================
// 4. FORWARD DECLARATIONS (REQUIRED FOR PLATFORMIO)
//========================================================================
void endNextionCmd();
void lockScreen();
void unlockScreen();
String getCurrentButtonID();
void highlightCurrentButton(int colorCode);
void clearAllButtonsOnPage();
void handleNext();
void handleSelect();
bool remoteIsPressed();
void readEncoder();
void interpretBedCommand(String cmd);
void readMPU();

//========================================================================
// 5. NEXTION HARDWARE COMMUNICATION HELPERS
//========================================================================
void endNextionCmd() {
  Serial2.write(0xFF); Serial2.write(0xFF); Serial2.write(0xFF);
}

void lockScreen() {
  Serial2.print("vis pBusy,1"); endNextionCmd();
  Serial2.print("tsw 255,0");   endNextionCmd();
}

void unlockScreen() {
  Serial2.print("vis pBusy,0"); endNextionCmd();
  Serial2.print("tsw 255,1");   endNextionCmd();
}

String getCurrentButtonID() {
  if (currentPage == 0) return page0_btns[currentIndex];
  if (currentPage == 1) return page1_btns[currentIndex];
  if (currentPage == 2) return page2_btns[currentIndex];
  return "";
}

void highlightCurrentButton(int colorCode) {
  String btn = getCurrentButtonID();
  if (btn != "") {
    Serial2.print(btn + ".bco=" + String(colorCode));
    endNextionCmd();
  }
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

//========================================================================
// 6. SETUP
//========================================================================
void setup() {
  Serial.begin(115200);                    // PC / Python Blinks
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Nextion TX/RX (ESP32 Pins 16/17)
  Wire.begin(4, 5);                        // I2C for MPU6050 (ESP32 Pins 4/5)

  delay(2000); // Allow Nextion to boot

  clearAllButtonsOnPage();
  highlightCurrentButton(hoverColor); 
}

//========================================================================
// 7. MAIN LOOP (NON-BLOCKING STATE MACHINE)
//========================================================================
void loop() {
  unsigned long currentMillis = millis();

  // --- A. PHYSICAL REMOTE & LOCKOUT LOGIC ---
  bool remoteActive = remoteIsPressed(); 

  if (remoteActive) { 
    if (!isScreenLocked) {
      lockScreen();          
      isScreenLocked = true;
      Serial.println("Remote active: Screen locked.");
    }
    lockoutStartTime = millis(); // Constantly reset stopwatch while held
  }

  // Check if remote is released AND 5 seconds have passed
  if (isScreenLocked && (millis() - lockoutStartTime >= lockoutDuration)) {
    unlockScreen();          
    isScreenLocked = false;
    Serial.println("Lockout ended: Screen unlocked.");
  }

  // --- B. READ ENCODER (10ms Loop) ---
  static unsigned long lastEncoderMillis = 0;
  if (currentMillis - lastEncoderMillis >= 10) {
    lastEncoderMillis = currentMillis;
    readEncoder();
  }

  // --- C. BLINK DETECTION (USB SERIAL FROM PYTHON) ---
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    
    // SAFETY CHECK: Only process blinks if screen is unlocked
    if (!isScreenLocked) {
      if (incomingChar == '0') {
        handleNext();           
        Serial.println("ACK:0");
      } else if (incomingChar == '1') {
        handleSelect();         
        Serial.println("ACK:1");
      }
    } else {
      Serial.println("Blink ignored: Remote lockout is active.");
    }
  }

  // --- D. NEXTION TOUCH SCREEN EVENTS (UART) ---
  if (Serial2.available() > 0) {
    String incomingCmd = Serial2.readStringUntil('\n');
    incomingCmd.trim();
    if (incomingCmd.length() > 0) {
      interpretBedCommand(incomingCmd);
    }
  }

  // --- E. MPU6050 PHYSICS ENGINE (20ms Loop) ---
  static unsigned long lastSampleMillis = 0;
  if (currentMillis - lastSampleMillis >= 20) {
    lastSampleMillis = currentMillis;
    readMPU();
    
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0;
    lastMicros = now;

    // TODO: Add your IMU math and PID motor driving logic here
  }
}

//========================================================================
// 8. ACTION HANDLERS & STUBS
//========================================================================
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

  // Simulate Nextion click (Press down, wait, release)
  Serial2.print("click " + target + ",1"); endNextionCmd();
  delay(100); 
  Serial2.print("click " + target + ",0"); endNextionCmd();

  // Track page state
  bool pageChanged = false;
  if (currentPage == 0) {
    if (target == "b1" || target == "b2") { currentPage = 1; pageChanged = true; }
    else if (target == "b0" || target == "b6") { currentPage = 2; pageChanged = true; }
  } 
  else if (currentPage == 1 || currentPage == 2) {
    if (target == "b0") { currentPage = 0; pageChanged = true; }
  }

  if (pageChanged) {
     currentIndex = 0;
     delay(200); 
     clearAllButtonsOnPage();
  }
  highlightCurrentButton(hoverColor); 
}

// --- HARDWARE LOGIC STUBS ---
// Replace the insides of these functions with your actual sensor/motor code

bool remoteIsPressed() {
  // TODO: Add your 74HC165 shift register read logic here
  return false; 
}

void readEncoder() {
  // TODO: Add your AS5600 logic here
}

void interpretBedCommand(String cmd) {
  // TODO: Add your motor command parser here
}

void readMPU() {
  // TODO: Add your Wire.h MPU6050 request logic here
}