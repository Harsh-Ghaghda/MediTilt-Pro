#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 4
#define SCL_PIN 5
#define PCA9548A_ADDR 0x70

void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10); // Give the switch gate time to settle
}

void disableAllChannels() {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n--- PCA9548A Mux-Aware I2C Scanner ---");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz standard speed
}

void loop() {
  // 1. First, check if the PCA9548A itself is alive on the main bus
  Wire.beginTransmission(PCA9548A_ADDR);
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.println("\n[ERROR] PCA9548A Multiplexer NOT detected at 0x70!");
    Serial.println("Check hardware:");
    Serial.println(" - A0, A1, A2 pins MUST be tied to GND.");
    Serial.println(" - RST (Reset) pin MUST be pulled HIGH to 3.3V (or left floating if module has pull-up).");
    Serial.println(" - Power: VCC to 3.3V, GND to GND.");
    delay(3000);
    return;
  }

  Serial.println("\nSUCCESS: PCA9548A Multiplexer detected at 0x70!");

  // 2. Scan through all 8 channels of the Multiplexer
  for (uint8_t chan = 0; chan < 8; chan++) {
    Serial.print("Scanning Channel ");
    Serial.print(chan);
    Serial.print("... ");

    selectMuxChannel(chan);
    int devicesOnChannel = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
      if (addr == PCA9548A_ADDR) continue; // Skip reporting the mux itself

      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        if (devicesOnChannel == 0) Serial.println();
        Serial.print("  -> Found device at address 0x");
        if (addr < 16) Serial.print("0");
        Serial.print(addr, HEX);

        if (addr == 0x68) Serial.print(" (MPU6050 Default)");
        if (addr == 0x69) Serial.print(" (MPU6050 AD0 High)");
        Serial.println();
        devicesOnChannel++;
      }
    }

    if (devicesOnChannel == 0) {
      Serial.println("No devices.");
    }
  }

  disableAllChannels();
  Serial.println("--- Scan Complete ---");
  delay(4000);
}