// ======================================================
// Arm.ino  -  ARM, GRIPPER (claw / suction), KIT SERVOS (PCA9685)
// ======================================================
#include "Config.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

float armBase, armBottom, armTop;
bool suctionActive = false;

void armApply() {
  pwm.setPWM(BASE_SERVO,   0, constrain((int)armBase,   BASE_MIN,   BASE_MAX));
  pwm.setPWM(BOTTOM_SERVO, 0, constrain((int)armBottom, BOTTOM_MIN, BOTTOM_MAX));
  pwm.setPWM(TOP_SERVO,    0, constrain((int)armTop,    TOP_MIN,    TOP_MAX));
}

// Smooth move. false = stopped (X / STOP button)
bool armMoveTo(int b, int bo, int t, unsigned long ms) {
  float b0 = armBase, bo0 = armBottom, t0 = armTop;
  float b1  = constrain(b,  BASE_MIN,   BASE_MAX);
  float bo1 = constrain(bo, BOTTOM_MIN, BOTTOM_MAX);
  float t1  = constrain(t,  TOP_MIN,    TOP_MAX);
  if ((int)b0 == (int)b1 && (int)bo0 == (int)bo1 && (int)t0 == (int)t1) return true;

  unsigned long start = millis();
  while (true) {
    if (abortRequested()) return false;
#if USE_MPU
    mpuUpdate();
#endif
    float k = ms ? (float)(millis() - start) / (float)ms : 1.0;
    if (k > 1.0) k = 1.0;
    float s = k * k * (3.0 - 2.0 * k);
    armBase   = b0  + (b1  - b0)  * s;
    armBottom = bo0 + (bo1 - bo0) * s;
    armTop    = t0  + (t1  - t0)  * s;
    armApply();
    if (k >= 1.0) break;
    delay(10);
  }
  armBase = b1; armBottom = bo1; armTop = t1;
  armApply();
  return true;
}

bool armToPose(const Pose3 &p, unsigned long ms) { return armMoveTo(p.b, p.bo, p.t, ms); }
bool armToInitial() { return armToPose(st.init, ARM_DOWN_MS); }
bool armToIdle()    { return armToPose(st.idle, ARM_DOWN_MS); }

// scan pose (camera looks down) with this base
bool armScanAt(int base) { return armMoveTo(base, st.scan.bo, st.scan.t, ARM_MOVE_MS); }
bool armLiftHere()       { return armMoveTo((int)armBase, st.scan.bo, st.scan.t, ARM_MOVE_MS); }

// point a little above a pick / place pose (30% back towards the scan pose)
bool armOver(float b, float bo, float t) {
  float hb = bo + HOVER_FRACTION * (st.scan.bo - bo);
  float ht = t  + HOVER_FRACTION * (st.scan.t  - t);
  return armMoveTo((int)(b + 0.5), (int)(hb + 0.5), (int)(ht + 0.5), ARM_MOVE_MS);
}

bool armDown(float b, float bo, float t) {
  return armMoveTo((int)(b + 0.5), (int)(bo + 0.5), (int)(t + 0.5), ARM_DOWN_MS);
}

// ---- CLAW servo (PCA9685 channel CLAW_CHANNEL) ----
void clawMoveDeg(int deg) {
  deg = constrain(deg, 0, 180);
  pwm.setPWM(CLAW_CHANNEL, 0, map(deg, 0, 180, CLAW_SERVO_MIN, CLAW_SERVO_MAX));
}

// ---- GRIPPER ----
// The whole program calls setSuction(true) = GRIP and setSuction(false) = RELEASE.
//   GRIPPER_CLAW 1 : true = claw CLOSE (clawClose deg)   false = claw OPEN (clawOpen deg)
//   GRIPPER_CLAW 0 : old suction pump (switch board on SUCTION_CHANNEL)
void setSuction(bool on) {
#if GRIPPER_CLAW
  // pump stays OFF (in case it is still connected)
  pwm.setPWM(SUCTION_CHANNEL, 0, (int)((long)SUCTION_PULSE_OFF_US * 4096L / 20000L));
  clawMoveDeg(on ? st.clawClose : st.clawOpen);
  suctionActive = on;
  Serial.println(on ? F("CLAW CLOSE (grip)") : F("CLAW OPEN"));
#else
  int us = on ? SUCTION_PULSE_ON_US : SUCTION_PULSE_OFF_US;
  pwm.setPWM(SUCTION_CHANNEL, 0, (int)((long)us * 4096L / 20000L));
  suctionActive = on;
  Serial.println(on ? F("Suction ON") : F("Suction OFF"));
#endif
}

// pause that can be stopped, keeps the gyro alive
bool waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (abortRequested()) return false;
#if USE_MPU
    mpuUpdate();
#endif
    delay(2);
  }
  return true;
}

// ---- kit servos: rest -> KICK -> rest ----
// each servo has its own REST + KICK angle (saved: st.kitRest / st.kitKick, command kitset)
static int kitPulse(int deg) {
  return map(constrain(deg, 0, 180), 0, 180, KIT_SERVO_MIN, KIT_SERVO_MAX);
}

byte kitChannel(Zone z) {
  if (z == ZONE_PCC1) return KIT_CH_PCC1;
  if (z == ZONE_PCC2) return KIT_CH_PCC2;
  return KIT_CH_HOSPITAL;
}

static byte kitIndex(Zone z) {           // 0 = hospital, 1 = PCC1, 2 = PCC2
  if (z == ZONE_PCC1) return 1;
  if (z == ZONE_PCC2) return 2;
  return 0;
}

// move one kit servo to any angle now (find the angles with this)
void kitMoveDeg(Zone z, int deg) {
  pwm.setPWM(kitChannel(z), 0, kitPulse(deg));
}

void kitsHoldAll() {
  kitMoveDeg(ZONE_HOSPITAL, st.kitRest[0]);
  kitMoveDeg(ZONE_PCC1,     st.kitRest[1]);
  kitMoveDeg(ZONE_PCC2,     st.kitRest[2]);
}

bool kitRelease(Zone z) {
  byte i = kitIndex(z);
  Serial.print(F(">>> KIT KICK: "));
  Serial.print(zoneName(z));
  Serial.print(F("  (channel "));
  Serial.print(kitChannel(z));
  Serial.print(F("  rest "));
  Serial.print(st.kitRest[i]);
  Serial.print(F("  kick "));
  Serial.print(st.kitKick[i]);
  Serial.println(F(")"));
  for (byte k = 0; k < KIT_KICK_TIMES; k++) {
    kitMoveDeg(z, st.kitKick[i]);
    if (!waitMs(KIT_KICK_MS)) { kitMoveDeg(z, st.kitRest[i]); return false; }
    kitMoveDeg(z, st.kitRest[i]);
    if (!waitMs(KIT_RETURN_MS)) return false;
  }
  return true;
}