#include <Wire.h>
#include <Arduino.h>
#include <Ticker.h>

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
#define ENCODER_SAMPLE_INTERVAL_MS 4   // 250 Hz - decoupled from the 20ms IMU/web loop

// volatile because these are now written from a timer interrupt (readEncoderISR)
// and read from the main loop - the compiler must not cache stale values.
//
// "rotations" is now an ABSOLUTE position count: it only ever sits at 0 or
// above. Moving away from the flat/reference position increases it, moving
// back toward flat decreases it, and it's clamped so it can never go
// negative - no separate labs() step needed anywhere else in the code.
volatile long rotations = 0;
volatile float lastAngle = 0;

Ticker encoderTicker;

// Runs on a hardware timer, completely independent of loop() timing (IMU
// processing, WiFi/web server handling, etc. can no longer starve this).
// Keep this short - no Serial prints, no String/malloc, no blocking calls.
void ICACHE_RAM_ATTR readEncoderISR()
{
  int raw = analogRead(ENCODER_PIN);
  float angle = (raw / 1023.0) * 360.0;

  float diff = angle - lastAngle;

  // NOTE: which branch means "away from flat" vs "toward flat" depends on
  // your physical wiring/geometry. This matches the direction observed in
  // rollmap.csv (abs_rotations increased steadily as the bed moved away
  // from flat). If your rotation count moves the wrong way, swap the ++ and
  // -- below.
  if (diff > 200.0) {
    rotations++;              // moving away from flat - grow the count
  }
  if (diff < -200.0) {
    rotations--;              // moving back toward flat
    if (rotations < 0) rotations = 0;  // clamp - stay on the positive scale
  }

  lastAngle = angle;
}

//========================
// Motor Control (Cytron MD30C R2) - drive to a target angle and stop
//========================
#include "calibration_table.h"

#define MOTOR_PWM_PIN 12   // D6 - Cytron PWM input (speed, sign-magnitude mode)
#define MOTOR_DIR_PIN 13   // D7 - Cytron DIR input (direction)

const int MOTOR_SPEED = 255;  // 0-255 PWM duty cycle - tune to a safe, controlled speed

long targetRotations = 0;
bool motorMoving = false;

// Converts an absolute rotation count to an interpolated roll angle, using
// the nonlinear calibration table (see calibration_table.h). Values outside
// the calibrated range are clamped to the nearest table endpoint.
float rotationsToRoll(long rot)
{
  int32_t rotFirst = pgm_read_dword(&calRotations[0]);
  int32_t rotLast  = pgm_read_dword(&calRotations[CAL_TABLE_SIZE - 1]);

  if (rot <= rotFirst) return pgm_read_float(&calRoll[0]);
  if (rot >= rotLast)  return pgm_read_float(&calRoll[CAL_TABLE_SIZE - 1]);

  for (int i = 0; i < CAL_TABLE_SIZE - 1; i++)
  {
    int32_t r0 = pgm_read_dword(&calRotations[i]);
    int32_t r1 = pgm_read_dword(&calRotations[i + 1]);
    if (rot >= r0 && rot <= r1)
    {
      float roll0 = pgm_read_float(&calRoll[i]);
      float roll1 = pgm_read_float(&calRoll[i + 1]);
      float t = (r1 == r0) ? 0.0f : (float)(rot - r0) / (float)(r1 - r0);
      return roll0 + t * (roll1 - roll0);
    }
  }
  return pgm_read_float(&calRoll[CAL_TABLE_SIZE - 1]); // shouldn't reach here
}

// Converts a target roll angle to the interpolated rotation count needed to
// reach it - the inverse lookup, using the same table. Works whether the
// table's roll values increase or decrease with rotation count. Values
// outside the calibrated range are clamped to the nearest table endpoint.
long rollToRotations(float targetRoll)
{
  float rollFirst = pgm_read_float(&calRoll[0]);
  float rollLast  = pgm_read_float(&calRoll[CAL_TABLE_SIZE - 1]);
  bool increasing = (rollLast >= rollFirst);

  if (increasing)
  {
    if (targetRoll <= rollFirst) return pgm_read_dword(&calRotations[0]);
    if (targetRoll >= rollLast)  return pgm_read_dword(&calRotations[CAL_TABLE_SIZE - 1]);
  }
  else
  {
    if (targetRoll >= rollFirst) return pgm_read_dword(&calRotations[0]);
    if (targetRoll <= rollLast)  return pgm_read_dword(&calRotations[CAL_TABLE_SIZE - 1]);
  }

  for (int i = 0; i < CAL_TABLE_SIZE - 1; i++)
  {
    float roll0 = pgm_read_float(&calRoll[i]);
    float roll1 = pgm_read_float(&calRoll[i + 1]);
    bool inRange = increasing
      ? (targetRoll >= roll0 && targetRoll <= roll1)
      : (targetRoll <= roll0 && targetRoll >= roll1);

    if (inRange)
    {
      int32_t rot0 = pgm_read_dword(&calRotations[i]);
      int32_t rot1 = pgm_read_dword(&calRotations[i + 1]);
      float t = (roll1 == roll0) ? 0.0f : (targetRoll - roll0) / (roll1 - roll0);
      return rot0 + (long)lround(t * (rot1 - rot0));
    }
  }
  return pgm_read_dword(&calRotations[CAL_TABLE_SIZE - 1]); // shouldn't reach here
}

void motorStop()
{
  analogWrite(MOTOR_PWM_PIN, 0);
}

// Drives the motor in whichever direction INCREASES the "rotations" count.
// If you type in a target and the bed tilts the wrong way, swap HIGH/LOW
// in these two functions (or swap the two motor wires on the Cytron output).
void motorDriveIncreasing()
{
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  analogWrite(MOTOR_PWM_PIN, MOTOR_SPEED);
}

void motorDriveDecreasing()
{
  digitalWrite(MOTOR_DIR_PIN, LOW);
  analogWrite(MOTOR_PWM_PIN, MOTOR_SPEED);
}

void setupMotor()
{
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  motorStop();
}

// Accumulates Serial characters until Enter, then parses the line as the
// target angle in degrees and arms a new move. Non-blocking - safe to call
// every loop iteration.
String angleInputBuffer = "";

void handleAngleSerialInput()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (angleInputBuffer.length() > 0)
      {
        float targetAngle = angleInputBuffer.toFloat();
        targetRotations = rollToRotations(targetAngle);
        motorMoving = true;

        Serial.print("New target angle: ");
        Serial.print(targetAngle, 2);
        Serial.print(" deg -> target rotations: ");
        Serial.println(targetRotations);

        angleInputBuffer = "";
      }
    }
    else
    {
      angleInputBuffer += c;
    }
  }
}

// Call once per sensor sample: steps the motor toward targetRotations and
// stops exactly when the encoder count matches it.
void updateMotorControl()
{
  if (!motorMoving) return;

  if (rotations > targetRotations)
  {
    motorDriveIncreasing();
  }
  else if (rotations < targetRotations)
  {
    motorDriveDecreasing();
  }
  else
  {
    motorStop();
    motorMoving = false;
    Serial.print("Target reached. Rotations: ");
    Serial.print(rotations);
    Serial.print("  Roll: ");
    Serial.println(roll, 2);
  }
}

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
// Calibration (5 seconds) - GYRO ONLY
//========================
void calibrateMPU()
{
  Serial.println();
  Serial.println("Calibrating MPU6050 Gyro... Keep platform perfectly still.");
  
  long gx = 0, gy = 0, gz = 0;
  const unsigned long calDuration = 5000; // 5 seconds
  const unsigned long sampleDelay = 5;    // ms between samples
  unsigned long startTime = millis();
  int samples = 0;

  while (millis() - startTime < calDuration)
  {
    readMPU();
    gx += GyX;
    gy += GyY;
    gz += GyZ;
    samples++;

    if (samples % 100 == 0) Serial.print(".");
    delay(sampleDelay);
  }
  Serial.println();

  // ONLY calibrate the gyro (zero-rate offset)
  gyroOffsetX = (float)gx / samples;
  gyroOffsetY = (float)gy / samples;
  gyroOffsetZ = (float)gz / samples;

  // Keep accelerometer offsets at 0 (or hardcode your known flat offsets here)
  accOffsetX = 0;
  accOffsetY = 0;
  accOffsetZ = 0;

  Serial.print("Calibration Complete. Samples: ");
  Serial.println(samples);
} 

//========================
// Setup
//========================
void setup()
{
  Serial.begin(115200);

  Wire.begin(4, 5); // SDA=D2 (GPIO4), SCL=D1 (GPIO5)

  // Set up the Cytron MD30C R2 motor driver pins
  setupMotor();

  setupMPU();
  delay(500);
  calibrateMPU();

// Seed the filters with initial positioning
  readMPU();
  float ax = AcX - accOffsetX;
  float ay = AcY - accOffsetY;
  float az = AcZ - accOffsetZ;

  cFilterRoll.angle  = atan2(ay, az) * 180.0 / PI;
  cFilterPitch.angle = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  
  lastMicros = micros();

  // --- NEW CODE: Seed the encoder count based on the true physical starting angle ---
  rotations = rollToRotations(cFilterRoll.angle);
  // --------------------------------------------------------------------------------

  // Seed encoder's lastAngle so the first ISR tick doesn't register a
  // false wraparound, then start the high-frequency encoder sampling.
  int raw = analogRead(ENCODER_PIN);
  lastAngle = (raw / 1023.0) * 360.0;
  
  encoderTicker.attach_ms(ENCODER_SAMPLE_INTERVAL_MS, readEncoderISR);
}

//========================
// Loop
//========================
void loop()

{
  // Read any pending target-angle input every iteration, regardless of
  // the sensor sample timer, so typing a command feels responsive.
  handleAngleSerialInput();

  // Run the sensor read / filter / broadcast block on a strict 20ms
  // cadence, without blocking Serial input via delay().
  static unsigned long lastSampleMillis = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastSampleMillis < 20)
  {
    return; // not time for the next sample yet - keep servicing the web stack
  }
  lastSampleMillis = nowMs;

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
  // No manual read here anymore - readEncoderISR() runs automatically in
  // the background via the Ticker at ENCODER_SAMPLE_INTERVAL_MS.
  // "rotations" is already an absolute, clamped-at-zero position count -
  // no labs() needed.

  //----- Drive the motor toward the target angle, if one is armed -----
  updateMotorControl();

  //----- Print combined output -----
  Serial.print("Roll: ");     Serial.print(roll, 2);
  Serial.print(" \tPitch: "); Serial.print(pitch, 2);
  Serial.print(" \tEncAngle: "); Serial.print(lastAngle, 2);
  Serial.print(" \tRotations: "); Serial.print(rotations);
  Serial.print(motorMoving ? "  [MOVING -> " : "  [stopped, target=");
  Serial.print(targetRotations);
  Serial.println("]");

  // Machine-parsable line for the pyserial logging script (rollmap_logger.py).
  // Format: DATA,<millis>,<abs_rotations>,<roll>
  Serial.print("DATA,");
  Serial.print(nowMs);
  Serial.print(",");
  Serial.print(rotations);
  Serial.print(",");
  Serial.println(roll, 2);
}