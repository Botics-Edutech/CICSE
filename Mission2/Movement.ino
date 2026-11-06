// ======================================================
// Movement.ino  -  WHEELS ONLY (encoders, motors, drive steps)
// Same as Mission 1. Added: M4_REVERSED (Mission-2 robot RR polarity).
// ======================================================
#include "Config.h"

#define RSIGN (ROTATION_MIRRORED ? -1 : 1)

byte stepCounter = 0;
bool lastMoveHitWall = false;
bool assistEnabled = true;
bool moveAllowStall = false;

volatile long enc[4] = {0, 0, 0, 0};
byte encPrev[4] = {0, 0, 0, 0};

const signed char QDEC[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

static inline void encStep(byte m, byte now) {
  enc[m] += QDEC[encPrev[m] * 4 + now];
  encPrev[m] = now;
}

// 20 kHz encoder sampler
ISR(TIMER1_COMPA_vect) {
  byte g = PING;
  byte l = PINL;

  encStep(0, g & 0x03);            // M1 FL  40/41
  encStep(1, (l / 64) & 0x03);     // M2 RL  42/43
  encStep(2, (l / 16) & 0x03);     // M3 FR  44/45
  encStep(3, (l / 4) & 0x03);      // M4 RR  46/47
}

void encodersBegin() {
  for (byte pin = 40; pin <= 47; pin++) {
    pinMode(pin, INPUT_PULLUP);
  }

  byte g = PING;
  byte l = PINL;

  encPrev[0] = g & 0x03;
  encPrev[1] = (l / 64) & 0x03;
  encPrev[2] = (l / 16) & 0x03;
  encPrev[3] = (l / 4) & 0x03;

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 799;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

void readEnc(long result[4]) {
  noInterrupts();
  for (byte i = 0; i < 4; i++) result[i] = enc[i];
  interrupts();
}

void resetEncoders() {
  noInterrupts();
  for (byte i = 0; i < 4; i++) enc[i] = 0;
  interrupts();
}

void motorRun(byte pwmPin, byte in1, byte in2, int speed) {
#if M4_REVERSED
  if (pwmPin == M4_PWM) speed = -speed;
#endif
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    analogWrite(pwmPin, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void stopMotors() {
  motorRun(M1_PWM, M1_IN1, M1_IN2, 0);
  motorRun(M2_PWM, M2_IN1, M2_IN2, 0);
  motorRun(M3_PWM, M3_IN1, M3_IN2, 0);
  motorRun(M4_PWM, M4_IN1, M4_IN2, 0);
}

void settleWait(unsigned long ms) {
#if USE_MPU
  unsigned long t0 = millis();
  while (millis() - t0 < ms) mpuUpdate();
#else
  delay(ms);
#endif
}

void brakeMotors(unsigned int ms) {
  if (ms == 0) {
    stopMotors();
    return;
  }

  digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, HIGH); analogWrite(M1_PWM, 255);
  digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, HIGH); analogWrite(M2_PWM, 255);
  digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, HIGH); analogWrite(M3_PWM, 255);
  digitalWrite(M4_IN1, HIGH); digitalWrite(M4_IN2, HIGH); analogWrite(M4_PWM, 255);

  settleWait(ms);
  stopMotors();
}

float moveProgress(MoveType type) {
#if USE_MPU
  if ((type == ROTATE_RIGHT || type == ROTATE_LEFT) && mpuOk) {
    mpuUpdate();
    float turned = yawDeg - moveYaw0;
    if (type == ROTATE_LEFT) turned = -turned;
    return turned * (TURN_90_WHEEL_MM * COUNTS_PER_MM / 90.0);
  }
#endif

  long current[4] = {0, 0, 0, 0};
  readEnc(current);

#if ENCODER_TOLERANT
  int8_t d[4];
  wheelDirs(type, d);
  long v[4];
  int n = 0;
  for (byte i = 0; i < 4; i++) if (d[i] != 0) v[n++] = labs(current[i]);
  for (int i = 0; i < n; i++)
    for (int j = i + 1; j < n; j++)
      if (v[j] > v[i]) { long t = v[i]; v[i] = v[j]; v[j] = t; }
  if (n >= 4) return (v[1] + v[2]) / 2.0;
  if (n == 3) return v[1];
  if (n == 2) return (v[0] + v[1]) / 2.0;
  return v[0];
#else
  if (type == DIAGONAL_FRONT_RIGHT) return (labs(current[0]) + labs(current[3])) / 2.0;
  return (labs(current[0]) + labs(current[1]) + labs(current[2]) + labs(current[3])) / 4.0;
#endif
}

// ---- gyro heading hold: P + I + D, with a memory ----
float assistI = 0;
float assistBias[7] = {0, 0, 0, 0, 0, 0, 0};
unsigned long assistLastMs = 0;
float assistSumAbs = 0, assistMaxAbs = 0;
unsigned int assistN = 0;
bool assistLog = false;
static unsigned long assistLogMs = 0;

static void assistStart(MoveType type) {
  assistI = assistBias[type];
  assistLastMs = millis();
  assistSumAbs = 0;
  assistMaxAbs = 0;
  assistN = 0;
  assistLogMs = 0;
}

static void assistLearn(MoveType type, unsigned long durMs) {
  if (type == ROTATE_RIGHT || type == ROTATE_LEFT) return;
  if (durMs < 600) return;
  assistBias[type] = constrain(assistI, -ASSIST_I_MAX, ASSIST_I_MAX);
}

void runMovementAssist(MoveType type, int spd) {
  if (type == ROTATE_RIGHT || type == ROTATE_LEFT) {
    runMovement(type, spd);
    return;
  }

  float c = 0;
#if USE_MPU
  if (mpuOk) {
    mpuUpdate();

    unsigned long nowMs = millis();
    float dtI = (nowMs - assistLastMs) / 1000.0;
    if (dtI > 0.1) dtI = 0.1;
    assistLastMs = nowMs;

    float err = yawDeg - headingRef;
    float pd = ASSIST_KP_PWM * err + ASSIST_KD_PWM * yawRate;
    float total = pd + assistI;

    if (assistEnabled && (fabs(total) < ASSIST_MAX_PWM || (total > 0) != (err > 0))) {
      assistI += ASSIST_KI_PWM * err * dtI;
      if (assistI >  ASSIST_I_MAX) assistI =  ASSIST_I_MAX;
      if (assistI < -ASSIST_I_MAX) assistI = -ASSIST_I_MAX;
      total = pd + assistI;
    }

    if (total >  ASSIST_MAX_PWM) total =  ASSIST_MAX_PWM;
    if (total < -ASSIST_MAX_PWM) total = -ASSIST_MAX_PWM;

    if (!assistEnabled) { total = 0; assistI = 0; }

    assistSumAbs += fabs(total);
    assistN++;
    if (fabs(total) > assistMaxAbs) assistMaxAbs = fabs(total);

    if (assistLog && nowMs - assistLogMs >= 150) {
      assistLogMs = nowMs;
      Serial.print(F("   gyro: heading error "));
      Serial.print(err);
      Serial.print(F(" deg   help "));
      Serial.print(total);
      Serial.print(F(" PWM   (I part "));
      Serial.print(assistI);
      Serial.println(F(")"));
    }

    c = total * ASSIST_SIGN;
  }
#endif

  float trim = 0;
  if (type == STRAFE_RIGHT) trim = STRAFE_RIGHT_FWD_TRIM_PWM;
  if (type == STRAFE_LEFT)  trim = STRAFE_LEFT_FWD_TRIM_PWM;

  if (c == 0 && trim == 0) {
    runMovement(type, spd);
    return;
  }

  int8_t d[4];
  wheelDirs(type, d);

  const int8_t rot[4] = {-RSIGN, -RSIGN, RSIGN, RSIGN};
  const byte pwmPin[4] = {M1_PWM, M2_PWM, M3_PWM, M4_PWM};
  const byte in1Pin[4] = {M1_IN1, M2_IN1, M3_IN1, M4_IN1};
  const byte in2Pin[4] = {M1_IN2, M2_IN2, M3_IN2, M4_IN2};

  for (byte i = 0; i < 4; i++) {
    if (d[i] == 0) {
      motorRun(pwmPin[i], in1Pin[i], in2Pin[i], 0);
    } else {
      float sp = d[i] * spd - c * rot[i] + trim;
      motorRun(pwmPin[i], in1Pin[i], in2Pin[i], (int)sp);
    }
  }
}

void runMovement(MoveType type, int spd) {
  int8_t d[4];
  wheelDirs(type, d);
  motorRun(M1_PWM, M1_IN1, M1_IN2, d[0] * spd);
  motorRun(M2_PWM, M2_IN1, M2_IN2, d[1] * spd);
  motorRun(M3_PWM, M3_IN1, M3_IN2, d[2] * spd);
  motorRun(M4_PWM, M4_IN1, M4_IN2, d[3] * spd);
}

// ---- battery-proof speed control ----
float refCps = REF_CPS_DEFAULT;

#define SPEED_EE_ADDR   400
#define SPEED_EE_MAGIC  0x5D78

void speedLoad() {
  uint16_t m = 0;
  float r = 0;
  EEPROM.get(SPEED_EE_ADDR, m);
  EEPROM.get(SPEED_EE_ADDR + 2, r);
  if (m == SPEED_EE_MAGIC && r > 200.0 && r < 30000.0) refCps = r;
  else                                                refCps = REF_CPS_DEFAULT;
}

void wheelDirs(MoveType type, int8_t d[4]) {
  switch (type) {
    case DIAGONAL_FRONT_RIGHT: d[0] =  1; d[1] =  0; d[2] =  0; d[3] =  1; break;
    case ROTATE_RIGHT:         d[0] = -RSIGN; d[1] = -RSIGN; d[2] = RSIGN; d[3] = RSIGN; break;
    case STRAFE_RIGHT:         d[0] =  1; d[1] = -1; d[2] = -1; d[3] =  1; break;
    case BACKWARD:             d[0] = -1; d[1] = -1; d[2] = -1; d[3] = -1; break;
    case STRAFE_LEFT:          d[0] = -1; d[1] =  1; d[2] =  1; d[3] = -1; break;
    case FORWARD:              d[0] =  1; d[1] =  1; d[2] =  1; d[3] =  1; break;
    case ROTATE_LEFT:          d[0] = RSIGN; d[1] = RSIGN; d[2] = -RSIGN; d[3] = -RSIGN; break;
  }
}

#if USE_MPU
static bool turnFine(MoveType type, float targetCounts) {
  float cpd = TURN_90_WHEEL_MM * COUNTS_PER_MM / 90.0;

  for (int k = 0; k < 12; k++) {
    settleWait(60);
    float errDeg = (targetCounts - moveProgress(type)) / cpd;
    if (fabs(errDeg) <= TURN_TOL_DEG) break;

    unsigned long ms = 15 + (unsigned long)(fabs(errDeg) * TURN_FINE_MS_PER_DEG);
    if (ms > 150) ms = 150;

    runMovement(type, errDeg > 0 ? TURN_FINE_PWM : -TURN_FINE_PWM);

    unsigned long t0 = millis();
    while (millis() - t0 < ms) {
      if (abortRequested()) return false;
      mpuUpdate();
    }
    brakeMotors(30);
  }
  return true;
}
#endif

// After a move: turn back to headingRef with small gyro-watched pulses.
bool fixHeading() {
#if USE_MPU && HEADING_FIX_ENABLE
  if (!mpuOk) return true;

  settleWait(120);

  for (int k = 0; k < 8; k++) {
    float errDeg = headingRef - yawDeg;
    if (fabs(errDeg) <= HEADING_FIX_TOL_DEG) break;

    if (fabs(errDeg) > 25.0) {
      Serial.println(F("HEADING: more than 25 degrees off, not corrected"));
      break;
    }

    Serial.print(F("   heading correction "));
    Serial.print(errDeg);
    Serial.println(F(" deg"));

    unsigned long ms = 15 + (unsigned long)(fabs(errDeg) * TURN_FINE_MS_PER_DEG);
    if (ms > 150) ms = 150;

    runMovement(errDeg > 0 ? ROTATE_RIGHT : ROTATE_LEFT, TURN_FINE_PWM);

    unsigned long t0 = millis();
    while (millis() - t0 < ms) {
      if (abortRequested()) return false;
      mpuUpdate();
    }

    brakeMotors(30);
    settleWait(80);
  }
#endif
  return true;
}

bool moveFinish(MoveType type, float targetCounts) {
  brakeMotors(BRAKE_MS);
  settleWait(150);

#if USE_MPU
  if ((type == ROTATE_RIGHT || type == ROTATE_LEFT) && mpuOk) {
    return turnFine(type, targetCounts);
  }
#endif

#if TRIM_ENABLE
  {
    float tol = TRIM_TOL_MM * COUNTS_PER_MM;
    float now = moveProgress(type);

    if (now > targetCounts + tol) {
      Serial.print(F("   trim: went "));
      Serial.print((now - targetCounts) / COUNTS_PER_MM);
      Serial.println(F(" mm too far, going back"));

      unsigned long t1 = millis();
      while (now > targetCounts + tol * 0.5 && millis() - t1 < TRIM_MAX_MS) {
        if (abortRequested()) return false;
        runMovement(type, -SLOW_SPEED);
        now = moveProgress(type);
      }
      brakeMotors(BRAKE_MS);
      settleWait(150);
    }
  }
#endif

  return true;
}

// SAFE way: fixed motor power + gyro help
bool encoderMoveOpen(float mm, float factor, MoveType type, float slowDistance) {
  resetEncoders();

  float targetCounts = mm * factor * COUNTS_PER_MM;

  unsigned long startTime = millis();
  unsigned long timeout = 4000UL + (unsigned long)(mm * 20.0);

  float progress = 0;
  float lastProg = 0;
  unsigned long lastProgTime = millis();

#if USE_MPU
  bool gyroTurn = mpuOk && (type == ROTATE_RIGHT || type == ROTATE_LEFT);
  float cpd = TURN_90_WHEEL_MM * COUNTS_PER_MM / 90.0;
#endif

  while (progress < targetCounts) {
    if (abortRequested()) return false;

    progress = moveProgress(type);

    if (progress - lastProg >= 8) {
      lastProg = progress;
      lastProgTime = millis();
    } else if (moveAllowStall && progress >= 0.6 * targetCounts && millis() - lastProgTime > 400) {
      Serial.println(F("   (wall reached)"));
      lastMoveHitWall = true;
      break;
    }

#if USE_MPU
    if (gyroTurn) {
      if (progress < -12.0 * cpd) {
        stopMotors();
        Serial.println(F("ERROR: the robot turns the WRONG WAY. Change ROTATION_MIRRORED in Config.h."));
        return false;
      }
      if (millis() - startTime > 2500 && progress < 3.0 * cpd) {
        stopMotors();
        Serial.println(F("ERROR: the robot does not turn. A wheel is not running (send E)."));
        return false;
      }
    }
#endif

    float remaining = targetCounts - progress;
    int currentSpeed = SPEED;
    if (remaining <= slowDistance * COUNTS_PER_MM) currentSpeed = SLOW_SPEED;
    // v17 soft start: power rises from RAMP_START_PWM to SPEED in RAMP_MS (wheels do not spin)
    {
      long el = (long)(millis() - startTime);
      int ramp = RAMP_START_PWM + (int)((long)(SPEED - RAMP_START_PWM) * el / RAMP_MS);
      if (ramp < currentSpeed) currentSpeed = ramp;
    }

    runMovementAssist(type, currentSpeed);

    if (millis() - startTime > timeout) {
      stopMotors();
      Serial.println(F("ERROR: MOVEMENT TIMEOUT"));
      return false;
    }
  }

  return moveFinish(type, targetCounts);
}

// ADVANCED way: per-wheel speed control + speed profile + gyro
bool encoderMoveClosed(float mm, float factor, MoveType type, float slowDistance) {
  resetEncoders();

  float targetCounts = mm * factor * COUNTS_PER_MM;

  int8_t d[4];
  wheelDirs(type, d);

  const int8_t rot[4] = {-RSIGN, -RSIGN, RSIGN, RSIGN};
  const byte pwmPin[4] = {M1_PWM, M2_PWM, M3_PWM, M4_PWM};
  const byte in1Pin[4] = {M1_IN1, M2_IN1, M3_IN1, M4_IN1};
  const byte in2Pin[4] = {M1_IN2, M2_IN2, M3_IN2, M4_IN2};

#if USE_MPU
  bool useGyro = mpuOk;
#else
  bool useGyro = false;
#endif
  bool isTurn = (type == ROTATE_RIGHT || type == ROTATE_LEFT);
  bool gyroTurn = useGyro && isTurn;

  float vMax   = refCps * ((slowDistance >= mm) ? V_SLOW_FRAC : V_MAX_FRAC);
  float vCreep = refCps * V_CREEP_FRAC;
  if (vMax < vCreep) vMax = vCreep;
  float acc = refCps * A_ACC_FRAC;
  float dec = refCps * A_DEC_FRAC;

  int active = 0;
  float pwmOut[4], vFilt[4];
  long prevAbs[4];
  for (byte i = 0; i < 4; i++) {
    pwmOut[i] = START_PWM;
    vFilt[i] = 0;
    prevAbs[i] = 0;
    if (d[i] != 0) active++;
  }

  float vCmd = refCps * 0.10;
  unsigned long startTime = millis();
  unsigned long timeout = 4000UL + (unsigned long)(mm * 20.0);
  unsigned long tLast = millis();

  while (true) {
    if (abortRequested()) return false;

    if (millis() - startTime > timeout) {
      stopMotors();
      Serial.println(F("ERROR: MOVEMENT TIMEOUT"));
      return false;
    }

#if USE_MPU
    if (useGyro) mpuUpdate();
#endif

    unsigned long now = millis();
    if (now - tLast < CTRL_MS) continue;

    float dt = (now - tLast) / 1000.0;
    tLast = now;

    long cur[4];
    readEnc(cur);

    float p[4];
    float avg = 0;
    for (byte i = 0; i < 4; i++) {
      p[i] = labs(cur[i]);
      if (d[i] != 0) avg += p[i];
    }
    avg /= active;

    float progress = avg;
    if (gyroTurn) {
      float turnedP = yawDeg - moveYaw0;
      if (type == ROTATE_LEFT) turnedP = -turnedP;
      progress = turnedP * (TURN_90_WHEEL_MM * COUNTS_PER_MM / 90.0);

      if (turnedP < -12.0) {
        stopMotors();
        Serial.println(F("ERROR: the robot turns the WRONG WAY. Run E and check the motors."));
        return false;
      }
      if (now - startTime > 2000 && fabs(turnedP) < 3.0) {
        stopMotors();
        Serial.println(F("ERROR: the robot does not turn. Run E (wheel test)."));
        return false;
      }
    }

    float remaining = targetCounts - progress;
    if (remaining <= 0) break;

    float vDec = sqrt(2.0 * dec * remaining) + vCreep;
    vCmd += acc * dt;

    float vTarget = vCmd;
    if (vTarget > vMax)  vTarget = vMax;
    if (vTarget > vDec)  vTarget = vDec;
    if (vTarget < vCreep) vTarget = vCreep;

    float corr = 0;
    if (useGyro && !isTurn) {
      float err = yawDeg - headingRef;
      float c = HEADING_KP * err + HEADING_KD * yawRate;
      float cMax = HEADING_MAX * vMax / refCps;
      if (c >  cMax) c =  cMax;
      if (c < -cMax) c = -cMax;
      corr = c * refCps;
    }

    for (byte i = 0; i < 4; i++) {
      if (d[i] == 0) {
        motorRun(pwmPin[i], in1Pin[i], in2Pin[i], 0);
        continue;
      }

      long a = labs(cur[i]);
      float cps = (float)(a - prevAbs[i]) / dt;
      prevAbs[i] = a;
      vFilt[i] += SPEED_FILTER * (cps - vFilt[i]);

      float tgt = vTarget + (useGyro ? SYNC_KP_GYRO : SYNC_KP) * (avg - p[i]);
      if (tgt < 0.7 * vTarget) tgt = 0.7 * vTarget;
      if (tgt > 1.3 * vTarget) tgt = 1.3 * vTarget;

      tgt += d[i] * (-corr * rot[i]);
      if (tgt < 0.0) tgt = 0.0;

      float denom = (tgt > 100.0) ? tgt : 100.0;
      float scale = (pwmOut[i] > 40.0) ? pwmOut[i] : 40.0;
      pwmOut[i] += SPEED_GAIN * (tgt - vFilt[i]) / denom * scale;
      if (pwmOut[i] > 255.0) pwmOut[i] = 255.0;
      if (pwmOut[i] < 0.0)   pwmOut[i] = 0.0;

      motorRun(pwmPin[i], in1Pin[i], in2Pin[i], d[i] * (int)(pwmOut[i] + 0.5));
    }
  }

  return moveFinish(type, targetCounts);
}

bool encoderMove(float mm, float factor, MoveType type, float slowDistance) {
  lastMoveHitWall = false;
#if USE_MPU
  mpuUpdate();
  moveYaw0 = headingRef;
#endif

  bool wasWall = moveAllowStall;
  unsigned long moveT0 = millis();
  assistStart(type);

  bool ok;
#if SPEED_CONTROL
  ok = encoderMoveClosed(mm, factor, type, slowDistance);
#else
  ok = encoderMoveOpen(mm, factor, type, slowDistance);
#endif

  moveAllowStall = false;

  if (ok && !wasWall && assistEnabled) assistLearn(type, millis() - moveT0);

#if USE_MPU
  if (ok && type == ROTATE_RIGHT && mpuOk) headingRef += mm * factor * 90.0 / TURN_90_WHEEL_MM;
  if (ok && type == ROTATE_LEFT && mpuOk)  headingRef -= mm * factor * 90.0 / TURN_90_WHEEL_MM;
#endif

  return ok;
}

// Q: measure REF speed and save it (WHEELS MOVE ~30 cm forward)
void speedTest() {
  Serial.println(F("SPEED TEST: the robot drives FORWARD about 1 second. Keep 50 cm clear."));

  resetEncoders();
  unsigned long t0 = millis();
  runMovement(FORWARD, SPEED);

  while (millis() - t0 < 400) if (abortRequested()) return;

  long a[4], b[4];
  readEnc(a);
  unsigned long ta = millis();

  while (millis() - t0 < 1000) if (abortRequested()) return;

  readEnc(b);
  unsigned long tb = millis();
  brakeMotors(BRAKE_MS);
  stopMotors();

  float dt = (tb - ta) / 1000.0;
  float sum = 0;

  for (byte i = 0; i < 4; i++) {
    float cps = (float)(labs(b[i]) - labs(a[i])) / dt;
    sum += cps;
    Serial.print(F("  wheel "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(cps);
    Serial.println(F(" counts/s"));
  }

  float mean = sum / 4.0;
  if (mean < 200.0 || mean > 30000.0) {
    Serial.println(F("SPEED TEST failed: the wheels hardly moved. NOT saved."));
    return;
  }

  refCps = mean;
  uint16_t m = SPEED_EE_MAGIC;
  EEPROM.put(SPEED_EE_ADDR, m);
  EEPROM.put(SPEED_EE_ADDR + 2, refCps);

  Serial.print(F("REFERENCE saved: "));
  Serial.print(refCps);
  Serial.println(F(" counts/s"));
}

// E: WHEEL TEST. Robot on a STAND (wheels in the air).
void wheelTest() {
  const byte pwmPin[4] = {M1_PWM, M2_PWM, M3_PWM, M4_PWM};
  const byte in1Pin[4] = {M1_IN1, M2_IN1, M3_IN1, M4_IN1};
  const byte in2Pin[4] = {M1_IN2, M2_IN2, M3_IN2, M4_IN2};
  const char *names[4] = {"M1 front-left", "M2 rear-left", "M3 front-right", "M4 rear-right"};

  Serial.println();
  Serial.println(F("WHEEL TEST: each wheel runs FORWARD alone. Robot on a STAND."));
  Serial.println(F("Every wheel must spin in the direction that drives the robot FORWARD."));

  for (byte w = 0; w < 4; w++) {
    resetEncoders();
    motorRun(pwmPin[w], in1Pin[w], in2Pin[w], 130);

    unsigned long t0 = millis();
    while (millis() - t0 < 600) {
      if (abortRequested()) { stopMotors(); return; }
    }

    stopMotors();
    delay(300);

    long c[4];
    readEnc(c);

    Serial.print(names[w]);
    Serial.print(F(" ->  E1="));
    Serial.print(c[0]);
    Serial.print(F("  E2="));
    Serial.print(c[1]);
    Serial.print(F("  E3="));
    Serial.print(c[2]);
    Serial.print(F("  E4="));
    Serial.println(c[3]);

    long own = labs(c[w]);
    long others = 0;
    for (byte k = 0; k < 4; k++) if (k != w && labs(c[k]) > others) others = labs(c[k]);

    if (own >= 50 && others < own / 3) {
      Serial.print(F("   OK: turns, encoder counts "));
      Serial.println(c[w] > 0 ? F("UP (+)") : F("DOWN (-)"));
    } else if (own < 20 && others >= 50) {
      Serial.println(F("   WRONG PAIR: another encoder counted (encoder wires on wrong pins)."));
    } else if (own < 20) {
      Serial.println(F("   NO COUNTS: motor did not turn OR its encoder is dead."));
    } else {
      Serial.println(F("   CHECK: counts look odd."));
    }
  }
  Serial.println(F("If M4 spins BACKWARD while the others go forward: change M4_REVERSED in Config.h."));
  Serial.println(F("WHEEL TEST finished."));
}

const char *moveName(MoveType type) {
  switch (type) {
    case DIAGONAL_FRONT_RIGHT: return "DIAGONAL FRONT RIGHT";
    case ROTATE_RIGHT:         return "ROTATE RIGHT (wheel mm)";
    case ROTATE_LEFT:          return "ROTATE LEFT (wheel mm)";
    case STRAFE_RIGHT:         return "STRAFE RIGHT";
    case STRAFE_LEFT:          return "STRAFE LEFT";
    case FORWARD:              return "FORWARD";
    case BACKWARD:             return "BACKWARD";
  }
  return "";
}

float moveFactor(MoveType type) {
  switch (type) {
    case DIAGONAL_FRONT_RIGHT: return DIAGONAL_FACTOR;
    case STRAFE_RIGHT:         return st.strafeF;     // saved in EEPROM (command: fstrafe)
    case STRAFE_LEFT:          return st.strafeF;
    case FORWARD:              return st.fwdF;        // saved in EEPROM (command: ffwd)
    case BACKWARD:             return st.backF;       // saved in EEPROM (command: fback)
    default:                   return 1.0;
  }
}

// Small slow move (camera alignment)
bool smallMove(MoveType type, float mm) {
  Serial.print(F("   move: "));
  Serial.print(moveName(type));
  Serial.print(F(" "));
  Serial.println((int)(mm + 0.5));

  if (!encoderMove(mm, moveFactor(type), type, mm)) {
    stopMotors();
    return false;
  }
  settleWait(150);
  return true;
}

// One numbered mission step
bool drive(MoveType type, float mm, float slowDistance) {
  float factor = moveFactor(type);

  stepCounter++;
  unsigned long driveStart = millis();

  Serial.print(F("STEP "));
  Serial.print(stepCounter);
  Serial.print(F(": "));
  Serial.print(moveName(type));
  Serial.print(F(" "));
  Serial.print((int)mm);
#if USE_MPU
  Serial.println(mpuOk ? F("    [gyro ON]") : F("    [!!! GYRO OFF]"));
#else
  Serial.println();
#endif

  if (!encoderMove(mm, factor, type, slowDistance)) {
    stopMotors();
    Serial.print(F("STOPPED: STEP "));
    Serial.print(stepCounter);
    Serial.println(F(" FAILED"));
    return false;
  }

  long e4[4];
  readEnc(e4);
  Serial.print(F("   done in "));
  Serial.print(millis() - driveStart);
  Serial.print(F(" ms. target counts="));
  Serial.print((long)(mm * factor * COUNTS_PER_MM));
  Serial.print(F("  E1="));
  Serial.print(e4[0]);
  Serial.print(F(" E2="));
  Serial.print(e4[1]);
  Serial.print(F(" E3="));
  Serial.print(e4[2]);
  Serial.print(F(" E4="));
  Serial.print(e4[3]);
#if USE_MPU
  if (mpuOk) {
    mpuUpdate();
    Serial.print(F("  heading error "));
    Serial.print(yawDeg - headingRef);
    Serial.print(F(" deg"));
    if (assistN > 0) {
      Serial.print(F("  gyro help avg "));
      Serial.print(assistSumAbs / assistN);
      Serial.print(F(" max "));
      Serial.print(assistMaxAbs);
    }
  }
#endif
  Serial.println();

  settleWait(300);
  return true;
}