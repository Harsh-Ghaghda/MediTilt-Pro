#include <Arduino.h>
#include <Wire.h>

#define BNO055_ADDR   0x28
#define TCA9548A_ADDR 0x70

#define CHANNEL_HEAD  2   // BNO on mux channel 2 -> head angle
#define CHANNEL_LEG   7   // BNO on mux channel 7 -> leg angle

void tcaSelect(uint8_t channel)
{
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void writeRegister(byte reg, byte value)
{
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte readRegister(byte reg)
{
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BNO055_ADDR, 1);
  return Wire.read();
}

int16_t read16(byte reg)
{
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BNO055_ADDR, 2);

  byte LSB = Wire.read();
  byte MSB = Wire.read();

  return (int16_t)((MSB << 8) | LSB);
}

void initBNO()
{
  writeRegister(0x3D, 0x00); // CONFIG mode
  delay(30);
  writeRegister(0x3E, 0x00); // normal power mode
  delay(30);
  writeRegister(0x3D, 0x0C); // NDOF mode
  delay(30);
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(4, 5); // SDA, SCL to mux

  Serial.println("Initializing BNO055s via TCA9548A...");

  tcaSelect(CHANNEL_HEAD);
  initBNO();
  Serial.println("Head BNO initialized (channel 2)");

  tcaSelect(CHANNEL_LEG);
  initBNO();
  Serial.println("Leg BNO initialized (channel 7)");
}

void loop()
{
  // Head angle (channel 2)
  tcaSelect(CHANNEL_HEAD);
  int16_t headRaw = read16(0x1E); // pitch register
  float headAngle = headRaw / 16.0;

  // Leg angle (channel 7)
  tcaSelect(CHANNEL_LEG);
  int16_t legRaw = read16(0x1E); // pitch register
  float legAngle = legRaw / 16.0;

  Serial.print("Head Angle: ");
  Serial.print(headAngle);
  Serial.print("  Leg Angle: ");
  Serial.println(legAngle);

  delay(20);
}