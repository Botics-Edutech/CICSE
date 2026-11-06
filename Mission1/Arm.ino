// ======================================================
// Arm.ino  -  ARM + SUCTION + SLAB SERVO (PCA9685)
// ======================================================
#include "Config.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

float armBase, armBottom, armTop;
bool armDriverOk = false;
bool suctionActive = false;

void armApply() {
  pwm.setPWM(BASE_SERVO,   0, constrain((int)armBase,   BASE_MIN,   BASE_MAX));
  pwm.setPWM(BOTTOM_SERVO, 0, constrain((int)armBottom, BOTTOM_MIN, BOTTOM_MAX));
  pwm.setPWM(TOP_SERVO,    0, constrain((int)armTop,    TOP_MIN,    TOP_MAX));
}

// Smooth move. Returns false if X was sent during the move.
bool armMoveTo(int b, int bo, int t, unsigned long durationMs) {
  float b0 = armBase, bo0 = armBottom, t0 = armTop;

  float b1  = constrain(b,  BASE_MIN,   BASE_MAX);
  float bo1 = constrain(bo, BOTTOM_MIN, BOTTOM_MAX);
  float t1  = constrain(t,  TOP_MIN,    TOP_MAX);

  // Already there: nothing to do
  if ((int)b0 == (int)b1 && (int)bo0 == (int)bo1 && (int)t0 == (int)t1) {
    return true;
  }

  unsigned long start = millis();

  while (true) {
    if (abortRequested()) {
      return false;
    }

    float k = (durationMs == 0) ? 1.0 : (float)(millis() - start) / (float)durationMs;
    if (k > 1.0) k = 1.0;

    float s = k * k * (3.0 - 2.0 * k);   // smoothstep

    armBase   = b0  + (b1  - b0)  * s;
    armBottom = bo0 + (bo1 - bo0) * s;
    armTop    = t0  + (t1  - t0)  * s;
    armApply();

    if (k >= 1.0) break;
    delay(10);
  }

  armBase = b1;
  armBottom = bo1;
  armTop = t1;
  armApply();
  return true;
}

// ---- named poses ----
bool armToInitial() { return armMoveTo(INITIAL_BASE, INITIAL_BOTTOM, INITIAL_TOP, ARM_DOWN_MS); }

// Scan / carry pose, arm pointing straight, camera facing down
bool armToScan()    { return armMoveTo(SCAN_BASE, SCAN_BOTTOM, SCAN_TOP, ARM_MOVE_MS); }

// Lift up to the scan height WITHOUT turning
bool armLiftHere()  { return armMoveTo((int)armBase, SCAN_BOTTOM, SCAN_TOP, ARM_MOVE_MS); }

// Go to a point a little ABOVE a pick / place pose: HOVER_FRACTION of the way back towards the SCAN pose
bool armOver(float base, float bottom, float top) {
  float scanTop = (SCAN_TOP > TOP_MIN) ? SCAN_TOP : TOP_MIN;
  float hb = bottom + HOVER_FRACTION * (SCAN_BOTTOM - bottom);
  float ht = top    + HOVER_FRACTION * (scanTop - top);
  return armMoveTo((int)(base + 0.5), (int)(hb + 0.5), (int)(ht + 0.5), ARM_MOVE_MS);
}

// Go down onto a pick / place pose
bool armDown(float base, float bottom, float top) {
  return armMoveTo((int)(base + 0.5), (int)(bottom + 0.5), (int)(top + 0.5), ARM_DOWN_MS);
}

// ---- suction (PCA9685 channel, full ON / full OFF) ----
void setSuction(bool on) {
  bool level = SUCTION_INVERT ? !on : on;

#if SUCTION_SERVO_SWITCH
  // pump switch board: a 50 Hz servo-like pulse (long pulse = ON, short pulse = OFF)
  int us = level ? SUCTION_PULSE_ON_US : SUCTION_PULSE_OFF_US;
  int counts = (int)((long)us * 4096L / 20000L);
  pwm.setPWM(SUCTION_CHANNEL, 0, counts);
#elif SUCTION_USE_MEGA_PIN
  digitalWrite(SUCTION_PIN, level ? HIGH : LOW);
#else
  // Pump noise can corrupt an I2C message, so OFF is sent 3 times
  // (and ON twice) to be sure it arrives.
  int times = on ? 2 : 3;

  for (int i = 0; i < times; i++) {
    if (level) pwm.setPWM(SUCTION_CHANNEL, 4096, 0);   // full ON
    else       pwm.setPWM(SUCTION_CHANNEL, 0, 4096);   // full OFF
    if (i < times - 1) delay(20);
  }
#endif

  suctionActive = on;
  Serial.println(on ? F("Suction ON") : F("Suction OFF"));
}

// Pause that can still be stopped with X
bool waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (abortRequested()) return false;
    delay(5);
  }
  return true;
}

// ---- slab servo ----
bool openSlab() {
  Serial.println();
  Serial.println(F(">>> SLAB SERVO"));

#if USE_SLAB_SERVO
  pwm.setPWM(SLAB_SERVO, 0, SLAB_OPEN);
  Serial.println(F("Slab servo OPEN"));
#else
  Serial.println(F("Slab servo NOT CONFIGURED (USE_SLAB_SERVO = 0). Pausing only."));
#endif

  return waitMs(SLAB_WAIT_MS);
}

void slabTest() {
#if USE_SLAB_SERVO
  static bool open = false;
  open = !open;
  pwm.setPWM(SLAB_SERVO, 0, open ? SLAB_OPEN : SLAB_CLOSED);
  Serial.println(open ? F("Slab servo OPEN") : F("Slab servo CLOSED"));
#else
  Serial.println(F("Slab servo NOT CONFIGURED. Set SLAB_SERVO, SLAB_CLOSED, SLAB_OPEN, then USE_SLAB_SERVO 1."));
#endif
}
