#include <Arduino.h>
// ============================================================================
// 74HC165 Remote Control Diagnostic
// ============================================================================
// Captures raw and debounced shifts to identify noise patterns

#define LATCH_PIN_REMOTE 27
#define CLOCK_PIN_REMOTE 14
#define DATA_PIN_REMOTE  35

// ---- Sampling and debounce ----
#define SHIFT_SAMPLES    5      // Take 5 consecutive reads
#define DEBOUNCE_MATCHES 3      // Need 3/5 to agree
#define DIAGNOSTIC_MS    500    // Print every 500ms

unsigned long lastDiagPrint = 0;
uint8_t sampleBuffer[SHIFT_SAMPLES];
uint8_t sampleIdx = 0;

byte read74HC165() {
  byte value = 0;
  for (int i = 0; i < 8; ++i) {
    int bitValue = digitalRead(DATA_PIN_REMOTE);
    value |= (bitValue << (7 - i));
    digitalWrite(CLOCK_PIN_REMOTE, HIGH);
    delayMicroseconds(1);
    digitalWrite(CLOCK_PIN_REMOTE, LOW);
  }
  return value;
}

byte readRemoteDebouncedRaw() {
  // Latch the current state
  digitalWrite(LATCH_PIN_REMOTE, LOW);
  delayMicroseconds(5);
  digitalWrite(LATCH_PIN_REMOTE, HIGH);
  delayMicroseconds(5);
  
  // Read
  byte raw = read74HC165();
  
  // Store in buffer
  sampleBuffer[sampleIdx] = raw;
  sampleIdx = (sampleIdx + 1) % SHIFT_SAMPLES;
  
  // Majority voting: find the value that appears most often
  uint8_t counts[256] = {0};
  for (int i = 0; i < SHIFT_SAMPLES; i++) {
    counts[sampleBuffer[i]]++;
  }
  
  uint8_t maxCount = 0;
  uint8_t stableValue = 0;
  for (int v = 0; v < 256; v++) {
    if (counts[v] > maxCount) {
      maxCount = counts[v];
      stableValue = v;
    }
  }
  
  // Only return the debounced value if we have enough agreement
  if (maxCount >= DEBOUNCE_MATCHES) {
    return stableValue;
  }
  return 0xFF;  // Sentinel for "unstable"
}

void printRemoteState(uint8_t state) {
  if (state == 0xFF) {
    Serial.println("  [UNSTABLE - no consensus]");
    return;
  }
  
  Serial.print("  Buttons: ");
  Serial.printf("HeadUp=%d ", bitRead(state, 3));
  Serial.printf("LegUp=%d ", bitRead(state, 2));
  Serial.printf("LeftUp=%d ", bitRead(state, 6));
  Serial.printf("RightUp=%d ", bitRead(state, 1));
  Serial.printf("HeadDn=%d ", bitRead(state, 4));
  Serial.printf("LegDn=%d ", bitRead(state, 5));
  Serial.printf("Kill=%d ", bitRead(state, 7));
  Serial.printf("Home=%d\n", bitRead(state, 0));
  Serial.printf("  Raw: 0x%02X (binary: %s)\n", state, 
    String(state, BIN).c_str());
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== 74HC165 Remote Diagnostic ===\n");
  
  // Initialize shift register pins
  pinMode(LATCH_PIN_REMOTE, OUTPUT);
  pinMode(CLOCK_PIN_REMOTE, OUTPUT);
  pinMode(DATA_PIN_REMOTE, INPUT);
  
  digitalWrite(LATCH_PIN_REMOTE, HIGH);
  digitalWrite(CLOCK_PIN_REMOTE, LOW);
  
  Serial.println("Setup complete. Press buttons or wait for noise...\n");
}

void loop() {
  unsigned long now = millis();
  
  // Read with debouncing
  byte stableState = readRemoteDebouncedRaw();
  
  // Print diagnostic info every 500ms
  if (now - lastDiagPrint >= DIAGNOSTIC_MS) {
    lastDiagPrint = now;
    Serial.printf("[%lu ms] Debounced state:\n", now);
    printRemoteState(stableState);
    
    // Also show the raw buffer to see variance
    Serial.print("  Raw samples: ");
    for (int i = 0; i < SHIFT_SAMPLES; i++) {
      Serial.printf("0x%02X ", sampleBuffer[i]);
    }
    Serial.println();
    Serial.println();
  }
  
  delay(2);  // Small delay to reduce CPU load, but keep polling tight
}
