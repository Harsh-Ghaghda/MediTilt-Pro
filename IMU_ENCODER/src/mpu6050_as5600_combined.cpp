#include <Wire.h>
#include <Arduino.h>

//========================
// MPU6050 Config
//========================
#define MPU6050_ADDR 0x68

// Raw Sensor Variables
int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

// Sensitivity Scale Factors
// Accel: ±2g range  -> 16384 LSB/g
// Gyro:  ±250 dps    -> 131 LSB/(deg/s)
const float ACC_SCALE  = 16384.0;
const float GYRO_SCALE = 131.0;

// Offsets
float accOffsetX = 0;
float accOffsetY = 0;
float accOffsetZ = 0;

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

// Angles
float roll = 0;
float pitch = 0;

// Timing
unsigned long lastMicros = 0;

// Complementary Filter Class
class ComplementaryFilter
{
  public:
    // TUNING FACTOR 'alpha':
    // Closer to 1.0 = trust gyro more (smooth, but drifts over time).
    // Closer to 0.0 = trust accelerometer more (noisy, but no drift).
    float alpha = 0.98;
    float angle = 0.0;

    // gyroRate: deg/s from gyro (bias-corrected)
    // accAngle: angle computed from accelerometer (deg)
    // dt: elapsed time in seconds since last update
    float update(float gyroRate, float accAngle, float dt)
    {
      float gyroAngle = angle + gyroRate * dt;
      angle = alpha * gyroAngle + (1.0 - alpha) * accAngle;
      return angle;
    }
};

ComplementaryFilter cFilterRoll;
ComplementaryFilter cFilterPitch;

//========================
// AS5600 (Analog) Config
//========================
#define ENCODER_PIN A0

int rotations = 0;
float lastAngle = 0;

//========================
// MPU6050 Initialization
//========================
void setupMPU()
{
  // Wake up the MPU6050 (Register 0x6B, clear sleep bit)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Set accelerometer to ±2g (Register 0x1C)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Set gyro to ±250 dps (Register 0x1B)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

//========================
// Read MPU6050
//========================
void readMPU()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Data register start (ACCEL_XOUT_H)
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, 14, true);

  // MPU6050 is big-endian (High byte first, then Low byte)
  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature (2 bytes)
  GyX = (Wire.read() << 8) | Wire.read();
  GyY = (Wire.read() << 8) | Wire.read();
  GyZ = (Wire.read() << 8) | Wire.read();
}

//========================
// Calibration (5 seconds)
//========================
void calibrateMPU()
{
  Serial.println();
  Serial.println("Calibrating MPU6050... Keep platform perfectly still.");

  long ax = 0, ay = 0, az = 0;
  long gx = 0, gy = 0, gz = 0;

  const unsigned long calDuration = 5000; // 5 seconds
  const unsigned long sampleDelay = 5;    // ms between samples
  unsigned long startTime = millis();
  int samples = 0;

  while (millis() - startTime < calDuration)
  {
    readMPU();
    ax += AcX;
    ay += AcY;
    az += AcZ;
    gx += GyX;
    gy += GyY;
    gz += GyZ;
    samples++;

    if (samples % 100 == 0) Serial.print(".");
    delay(sampleDelay);
  }
  Serial.println();

  accOffsetX = (float)ax / samples;
  accOffsetY = (float)ay / samples;
  // ACC_SCALE corresponds to 1g of gravity vector at baseline flat position
  accOffsetZ = ((float)az / samples) - ACC_SCALE;

  gyroOffsetX = (float)gx / samples;
  gyroOffsetY = (float)gy / samples;
  gyroOffsetZ = (float)gz / samples;

  Serial.print("Calibration Complete. Samples: ");
  Serial.println(samples);
}

//========================
// AS5600 Read (Analog)
//========================
void readEncoder()
{
  int raw = analogRead(ENCODER_PIN);
  float angle = (raw / 1023.0) * 360.0;

  float diff = angle - lastAngle;

  if (diff < -200.0) {
    rotations++;
  }
  if (diff > 200.0) {
    rotations--;
  }

  lastAngle = angle;
}

//========================
// Setup
//========================
void setup()
{
  Serial.begin(9600);

  Wire.begin(4, 5); // SDA=D2 (GPIO4), SCL=D1 (GPIO5)

  setupMPU();
  delay(500);
  calibrateMPU();

  // Seed the filters with initial flat positioning
  readMPU();
  float ax = AcX - accOffsetX;
  float ay = AcY - accOffsetY;
  float az = AcZ - accOffsetZ;

  cFilterRoll.angle  = atan2(ay, az) * 180.0 / PI;
  cFilterPitch.angle = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  lastMicros = micros();

  // Seed encoder's lastAngle so the first loop iteration doesn't
  // register a false wraparound
  int raw = analogRead(ENCODER_PIN);
  lastAngle = (raw / 1023.0) * 360.0;

  Serial.println("MPU6050 + AS5600 running side by side.");
}

//========================
// Loop
//========================
void loop()
{
  //----- MPU6050 -----
  readMPU();

  // Compute elapsed time since last update
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;

  // Strip out zero-point calibration offsets (accel)
  float ax = AcX - accOffsetX;
  float ay = AcY - accOffsetY;
  float az = AcZ - accOffsetZ;

  // Strip out zero-rate offsets and convert gyro to deg/s
  float gxRate = (GyX - gyroOffsetX) / GYRO_SCALE;
  float gyRate = (GyY - gyroOffsetY) / GYRO_SCALE;

  // Convert raw gravity vectors into physical geometric angles
  float rollAcc  = atan2(ay, az) * 180.0 / PI;
  float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Fuse gyro rate + accel angle via complementary filter
  // Note: roll relates to rotation about X axis -> use gyro X rate
  //       pitch relates to rotation about Y axis -> use gyro Y rate
  roll  = cFilterRoll.update(gxRate, rollAcc, dt);
  pitch = cFilterPitch.update(gyRate, pitchAcc, dt);

  //----- AS5600 (Analog) -----
  readEncoder();

  //----- Print combined output -----
  Serial.print("Roll: ");     Serial.print(roll, 2);
  Serial.print(" \tPitch: "); Serial.print(pitch, 2);
  Serial.print(" \tEncAngle: "); Serial.print(lastAngle, 2);
  Serial.print(" \tRotations: "); Serial.println(rotations);

  delay(20); // Steady ~50Hz sample processing rate for both sensors
}
