// ======================================================
// Mpu.ino  -  MPU6050 GYRO: keeps the robot straight and turns EXACTLY
//
// yawDeg  = how far the robot has turned since the calibration
//           (+ = turned RIGHT / clockwise)
// The robot must stand STILL while the gyro is calibrated (boot and START).
//
// If the yaw goes the wrong way when you turn the robot to the right,
// change GYRO_SIGN in Config.h (test with the M key).
// ======================================================
#include "Config.h"

float yawDeg = 0;        // + = turned right
float yawRate = 0;       // deg/s
float headingRef = 0;    // the heading the robot must keep
float moveYaw0 = 0;      // yaw at the start of a turn
bool  mpuOk = false;
uint8_t mpuAddress = MPU_ADDR;       // the address where the chip was found (0x68, or 0x69)
unsigned int mpuFailTotal = 0;       // gyro reads that failed (I2C noise)
unsigned int mpuGlitchTotal = 0;     // gyro readings that were a wild jump and were ignored
static byte failRun = 0;

static float gyroBias = 0;           // raw units
static unsigned long lastUs = 0;
static float lastRate = 0;
static bool lastValid = false;

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuReadRaw(int &raw) {
  Wire.beginTransmission(mpuAddress);
  Wire.write((uint8_t)(0x43 + 2 * GYRO_AXIS));
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)mpuAddress, (uint8_t)2) != 2) return false;
  int hi = Wire.read();
  int lo = Wire.read();
  raw = (int16_t)((hi << 8) | lo);
  return true;
}

bool mpuBegin() {
  mpuOk = false;

  // the chip answers on 0x68, or on 0x69 when its AD0 pin is high
  const uint8_t tryAddr[2] = {(uint8_t)MPU_ADDR, (uint8_t)(MPU_ADDR == 0x68 ? 0x69 : 0x68)};
  bool found = false;
  for (byte i = 0; i < 2 && !found; i++) {
    mpuAddress = tryAddr[i];
    Wire.beginTransmission(mpuAddress);
    found = (Wire.endTransmission() == 0);
  }
  if (!found) {
    mpuAddress = MPU_ADDR;
    return false;
  }

  mpuWrite(0x6B, 0x01);     // wake up, clock from the gyro
  delay(50);
  mpuWrite(0x1A, 0x03);     // low-pass filter 44 Hz
  mpuWrite(0x1B, 0x08);     // +-500 deg/s  (65.5 counts per deg/s)
  delay(10);

  mpuOk = true;
  return true;
}

static int calBuf[MPU_CAL_SAMPLES];

static int cmpInt(const void *a, const void *b) {
  int x = *(const int *)a;
  int y = *(const int *)b;
  return (x > y) - (x < y);
}

// Robot must stand still. Takes about 1.5 seconds.
// Wild readings (I2C noise) are thrown away: only the readings near the middle value are used.
bool mpuCalibrate() {
  if (!mpuOk) return false;

  int n = 0;
  for (int i = 0; i < MPU_CAL_SAMPLES; i++) {
    int raw;
    if (mpuReadRaw(raw)) calBuf[n++] = raw;
    delay(3);
  }

  if (n < MPU_CAL_SAMPLES / 2) {
    Serial.println(F("MPU: reading failed during calibration"));
    return false;
  }

  qsort(calBuf, n, sizeof(int), cmpInt);
  int median = calBuf[n / 2];

  float sum = 0;
  int a = 0;
  float mn = 1e9, mx = -1e9;

  for (int i = 0; i < n; i++) {
    if (abs(calBuf[i] - median) > 196) continue;      // more than 3 deg/s from the middle: a wild reading
    sum += calBuf[i];
    a++;
    if (calBuf[i] < mn) mn = calBuf[i];
    if (calBuf[i] > mx) mx = calBuf[i];
  }

  if (a < (n * 7) / 10) {
    Serial.println(F("WARNING: the gyro readings are too noisy / the robot moved during the calibration - NOT used. Keep it still."));
    return false;
  }

  if ((mx - mn) / 65.5 > 8.0) {
    Serial.println(F("WARNING: the robot was moving during the gyro calibration - NOT used. Keep it still."));
    return false;
  }

  gyroBias = sum / a;

  Serial.print(F("MPU calibrated. bias="));
  Serial.print(gyroBias / 65.5);
  Serial.print(F(" deg/s  noise="));
  Serial.print((mx - mn) / 65.5);
  Serial.print(F(" deg/s"));
  if (n - a > 0) {
    Serial.print(F("  ("));
    Serial.print(n - a);
    Serial.print(F(" wild readings ignored)"));
  }
  Serial.println();

  yawDeg = 0;
  yawRate = 0;
  lastValid = false;
  return true;
}

// Call this often while the robot moves
void mpuUpdate() {
  if (!mpuOk) return;

  unsigned long now = micros();
  if (lastValid && (now - lastUs) < 2000UL) return;

  int raw;
  if (!mpuReadRaw(raw)) {
    mpuFailTotal++;
    // many failed reads in a row: wake the chip up again (it may have lost its settings)
    if (++failRun >= 20) {
      failRun = 0;
      Serial.println(F("GYRO: I2C reads fail - restarting the gyro chip"));
      mpuWrite(0x6B, 0x01);
      mpuWrite(0x1A, 0x03);
      mpuWrite(0x1B, 0x08);
    }
    return;
  }
  failRun = 0;

  float rate = GYRO_SIGN * ((float)raw - gyroBias) / 65.5;

  // a wild jump between two readings (a few ms apart) is I2C noise: keep the last rate
  if (lastValid && fabs(rate - lastRate) > MPU_GLITCH_DPS) {
    mpuGlitchTotal++;
    rate = lastRate;
  }

  if (fabs(rate) < 0.15) rate = 0;          // dead-band: no creeping when still

  if (lastValid) {
    float dt = (now - lastUs) / 1000000.0;
    if (dt < 0.25) yawDeg += 0.5 * (rate + lastRate) * dt;
  }

  lastUs = now;
  lastRate = rate;
  yawRate = rate;
  lastValid = true;
}

// M: show the yaw for 10 seconds. Turn the robot to the RIGHT by hand:
// the yaw must go UP (+). If it goes down, change GYRO_SIGN.
void mpuTest() {
  if (!mpuOk) {
    Serial.println(F("MPU6050 NOT FOUND (I2C 0x68). Check wiring: VCC, GND, SDA, SCL."));
    return;
  }

  Serial.println(F("MPU TEST 10 s: turn the robot to the RIGHT by hand. Yaw must go UP (+)."));

  unsigned long t0 = millis();
  unsigned long tp = 0;

  while (millis() - t0 < 10000) {
    if (abortRequested()) return;
    mpuUpdate();

    if (millis() - tp >= 250) {
      tp = millis();
      Serial.print(F("yaw = "));
      Serial.print(yawDeg);
      Serial.println(F(" deg"));
    }
  }
}
