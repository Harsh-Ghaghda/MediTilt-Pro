#include <Arduino.h>
#include <Wire.h>

struct EncoderState;

// ================= MUX CONFIG =================
#define TCA9548A_ADDR 0x70

#define CHANNEL_HEAD_IMU     2   // BNO on mux channel 2 -> head angle
#define CHANNEL_LEG_IMU      7   // BNO on mux channel 7 -> leg angle
#define CHANNEL_HEAD_ENCODER 1   // AS5600 on mux channel 1 -> head motor
#define CHANNEL_LEG_ENCODER  0   // AS5600 on mux channel 0 -> leg motor  
#define CHANNEL_SIDE_ENCODER 3   // AS5600 on mux channel 3 -> side encoder
#define CHANNEL_RIGHT_IMU    4   // BNO on mux channel 4 -> right angle
#define CHANNEL_LEFT_IMU     6   // BNO on mux channel 6 -> left angle
// ===== ANGLE OFFSETS (degrees, edit as needed) =====
#define HEAD_OFFSET   178
#define LEG_OFFSET    0.0
#define RIGHT_OFFSET  173
#define LEFT_OFFSET   180
#define SIDE_SUBROTATIONS 5   // subrotations per full rotation 

void tcaSelect(uint8_t channel)
{
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ================= AS5600 ENCODER =================
#define AS5600_ADDR         0x36
#define AS5600_REG_STATUS   0x0B
#define AS5600_REG_RAWANGLE 0x0C   // 12-bit raw angle, 2 bytes

struct EncoderState {
    uint8_t muxChannel;
    int rotations;
    int16_t lastRaw;
    bool firstRead;
};

EncoderState headEncoder = { CHANNEL_HEAD_ENCODER, 0, 0, true };
EncoderState legEncoder  = { CHANNEL_LEG_ENCODER,  0, 0, true };
EncoderState sideEncoder = { CHANNEL_SIDE_ENCODER, 0, 0, true };

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

float updateEncoder(EncoderState &enc)
{
  tcaSelect(enc.muxChannel);

  uint16_t rawAngle = as5600Read12(AS5600_REG_RAWANGLE);
  float degrees = (rawAngle / 4096.0) * 360.0;

  if (!enc.firstRead) {
    int16_t diff = (int16_t)rawAngle - enc.lastRaw;

    // Wrap detection in raw counts (12-bit, 0-4095), 2048 = half scale
    if (diff < -2048) {
      enc.rotations--;
    } else if (diff > 2048) {
      enc.rotations++;
    }
  }
  enc.firstRead = false;
  enc.lastRaw = (int16_t)rawAngle;

  return degrees;
}

// ================= BNO055 IMU (unchanged) =================
#define BNO055_ADDR   0x28

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

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);
  Wire.begin(4, 5); // SDA, SCL to mux

  Serial.println("Initializing BNO055s via TCA9548A...");
  tcaSelect(CHANNEL_HEAD_IMU);
  initBNO();
  Serial.println("Head BNO initialized (channel 2)");

  tcaSelect(CHANNEL_LEG_IMU);
  initBNO();
  Serial.println("Leg BNO initialized (channel 7)");

  tcaSelect(CHANNEL_RIGHT_IMU);
 initBNO();
 Serial.println("Right BNO initialized (channel 4)");

 tcaSelect(CHANNEL_LEFT_IMU);
 initBNO();
 Serial.println("Left BNO initialized (channel 6)");

  Serial.println("Checking AS5600 encoders...");

  tcaSelect(CHANNEL_HEAD_ENCODER);
  delay(10);
  Serial.print("Head encoder (channel 1) magnet: ");
  Serial.println(as5600MagnetDetected() ? "OK" : "NOT DETECTED");

  tcaSelect(CHANNEL_LEG_ENCODER);
  delay(10);
  Serial.print("Leg encoder (channel 0) magnet: ");
  Serial.println(as5600MagnetDetected() ? "OK" : "NOT DETECTED");
  tcaSelect(CHANNEL_SIDE_ENCODER);
  delay(10);
  Serial.print("Side encoder (channel 3) magnet: ");
  Serial.println(as5600MagnetDetected() ? "OK" : "NOT DETECTED");
}

// ================= LOOP =================
void loop()
{
  // ---- Encoder readings (I2C via mux) ----
  float headEncAngle = updateEncoder(headEncoder);
  float legEncAngle  = updateEncoder(legEncoder);
  float sideEncAngle = updateEncoder(sideEncoder);

  Serial.print("Head Motor -> Angle: ");
  Serial.print(headEncAngle);
  Serial.print(" | Rotations: ");
  Serial.print(headEncoder.rotations);

  Serial.print("   ||   Leg Motor -> Angle: ");
  Serial.print(legEncAngle);
  Serial.print(" | Rotations: ");
  Serial.print(legEncoder.rotations);

  int sideSub = sideEncoder.rotations * SIDE_SUBROTATIONS
              + (int)(sideEncAngle / (360.0 / SIDE_SUBROTATIONS));

  Serial.print("   ||   Side -> Angle: ");
  Serial.print(sideEncAngle);
  Serial.print(" | Subrotations: ");
  Serial.print(sideSub);

  // ---- IMU readings (I2C via mux) ----
   tcaSelect(CHANNEL_HEAD_IMU);
  int16_t headRaw = read16(0x1E);
  float headAngle = (headRaw / 16.0) + HEAD_OFFSET;   // inverted

  tcaSelect(CHANNEL_LEG_IMU);
  int16_t legRaw = read16(0x1E);
  float legAngle = (legRaw / 16.0) + LEG_OFFSET;

  tcaSelect(CHANNEL_RIGHT_IMU);
  float rightAngle = -(read16(0x1E) / 16.0) + RIGHT_OFFSET;

  tcaSelect(CHANNEL_LEFT_IMU);
  float leftAngle = (read16(0x1E) / 16.0) + LEFT_OFFSET;

 Serial.print("  Head Angle: ");
 Serial.print(headAngle);
 Serial.print("  Leg Angle: ");
 Serial.print(legAngle);
 Serial.print("  Right Angle: ");
 Serial.print(rightAngle);
 Serial.print("  Left Angle: ");
 Serial.println(leftAngle);
  delay(2);
}
