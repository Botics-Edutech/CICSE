// ======================================================
// Mpu.ino  -  MPU6050 GYRO (same as Mission 1)
//
// yawDeg  = how far the robot has turned since the calibration
//           (+ = turned RIGHT / clockwise)
// The robot must stand STILL while the gyro is calibrated (boot and START).
// If the yaw goes the wrong way when you turn the robot to the right,
// change GYRO_SIGN in Config.h (test with the M key).
// ======================================================
#include "Config.h"

float yawDeg = 0;
float yawRate = 0;
float headingRef = 0;
float moveYaw0 = 0;
bool  mpuOk = false;
uint8_t mpuAddress = MPU_ADDR;
unsigned int mpuFailTotal = 0;
unsigned int mpuGlitchTotal = 0;
static byte failRun = 0;

static float gyroBias = 0;
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

  mpuWrite(0x6B, 0x01);
  delay(50);
  mpuWrite(0x1A, 0x03);
  mpuWrite(0x1B, 0x08);
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
    if (abs(calBuf[i] - median) > 196) continue;
    sum += calBuf[i];
    a++;
    if (calBuf[i] < mn) mn = calBuf[i];
    if (calBuf[i] > mx) mx = calBuf[i];
  }

  if (a < (n * 7) / 10) {
    Serial.println(F("WARNING: gyro too noisy / robot moved during calibration - NOT used. Keep it still."));
    return false;
  }

  if ((mx - mn) / 65.5 > 8.0) {
    Serial.println(F("WARNING: robot was moving during gyro calibration - NOT used. Keep it still."));
    return false;
  }

  gyroBias = sum / a;

  Serial.print(F("MPU calibrated. bias="));
  Serial.print(gyroBias / 65.5);
  Serial.print(F(" deg/s  noise="));
  Serial.print((mx - mn) / 65.5);
  Serial.println(F(" deg/s"));

  yawDeg = 0;
  yawRate = 0;
  lastValid = false;
  return true;
}

void mpuUpdate() {
  if (!mpuOk) return;

  unsigned long now = micros();
  if (lastValid && (now - lastUs) < 2000UL) return;

  int raw;
  if (!mpuReadRaw(raw)) {
    mpuFailTotal++;
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

  if (lastValid && fabs(rate - lastRate) > MPU_GLITCH_DPS) {
    mpuGlitchTotal++;
    rate = lastRate;
  }

  if (fabs(rate) < 0.15) rate = 0;

  if (lastValid) {
    float dt = (now - lastUs) / 1000000.0;
    if (dt < 0.25) yawDeg += 0.5 * (rate + lastRate) * dt;
  }

  lastUs = now;
  lastRate = rate;
  yawRate = rate;
  lastValid = true;
}

// M: show the yaw for 10 s. Turn the robot RIGHT by hand: yaw must go UP (+).
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