#include <Arduino.h>
#include <Wire.h>

// ================= MUX CONFIG =================
#define TCA9548A_ADDR 0x70
#define CHANNEL_ENCODER 1   // AS5600 on mux channel 1

// ================= AS5600 CONFIG =================
#define AS5600_ADDR      0x36
#define AS5600_REG_STATUS   0x0B
#define AS5600_REG_RAWANGLE 0x0C   // 12-bit raw angle, 2 bytes (0x0C=high, 0x0D=low)
#define AS5600_REG_ANGLE    0x0E   // 12-bit filtered angle, 2 bytes

int rotations = 0;
int16_t lastRaw = 0;
bool firstRead = true;

void tcaSelect(uint8_t channel)
{
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

uint16_t as5600Read12(uint8_t reg)
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);

  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();

  return ((hi << 8) | lo) & 0x0FFF;  // 12-bit value, 0-4095
}

bool as5600MagnetDetected()
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_REG_STATUS);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 1);
  uint8_t status = Wire.read();
  return (status & 0x20) != 0;  // MD bit
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(4, 5); // SDA, SCL to mux

  tcaSelect(CHANNEL_ENCODER);
  delay(10);

  if (as5600MagnetDetected()) {
    Serial.println("AS5600 magnet detected on mux channel 1");
  } else {
    Serial.println("WARNING: AS5600 magnet NOT detected on mux channel 1");
  }
}

void loop()
{
  tcaSelect(CHANNEL_ENCODER);

  uint16_t rawAngle = as5600Read12(AS5600_REG_RAWANGLE);
  float degrees = (rawAngle / 4096.0) * 360.0;

  if (!firstRead) {
    int16_t diff = (int16_t)rawAngle - lastRaw;

    // Wrap detection in raw counts (12-bit, 0-4095).
    // Half of full scale = 2048 as the crossing threshold.
    if (diff < -2048) {
      rotations++;
    } else if (diff > 2048) {
      rotations--;
    }
  }
  firstRead = false;
  lastRaw = (int16_t)rawAngle;

  Serial.print("Encoder Angle: ");
  Serial.print(degrees);
  Serial.print(" deg | Raw: ");
  Serial.print(rawAngle);
  Serial.print(" | Rotations: ");
  Serial.println(rotations);

  delay(5);
}
