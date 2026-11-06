// ======================================================
// Mission.ino  -  REPLAY: pick each patient exactly where it was taught
// ======================================================
#include "Config.h"

// ---- DROP: the patient must let go at once ----
#ifndef RELEASE_WAIT_MS
#define RELEASE_WAIT_MS   1500   // arm stays DOWN this long after suction OFF
#endif
#ifndef RELEASE_WIGGLE
#define RELEASE_WIGGLE       3   // small base shakes to break the vacuum (0 = off)
#endif
#ifndef RELEASE_WIGGLE_UNITS
#define RELEASE_WIGGLE_UNITS 4
#endif
#ifndef PLACE_LIFT_MS
#define PLACE_LIFT_MS     1500   // slow lift after the drop
#endif

unsigned long missionT0 = 0;
byte dropCount[ZONE_COUNT] = {0, 0, 0, 0};
byte patientsDone = 0;

void missionReset() {
  stepCounter = 0;
  missionAborted = false;
  patientsDone = 0;
  navX = 0;
  navY = 0;
  for (byte z = 0; z < ZONE_COUNT; z++) dropCount[z] = 0;
  missionT0 = millis();
}

// gyro must be there; calibrate while standing still. waitTotalMs = whole wait
bool gyroStart(unsigned long waitTotalMs) {
#if USE_MPU
  if (!mpuOk) mpuBegin();
#if MPU_REQUIRED
  if (!mpuOk) {
    Serial.println(F("!!! GYRO (MPU6050) not found - not started. Send i2c."));
    return false;
  }
#endif
#endif
  Serial.println(F("Gyro calibrating - do not touch the robot..."));
  unsigned long w0 = millis();
  if (!waitMs(600)) return false;
#if USE_MPU
  bool cal = false;
  for (int i = 0; i < 3 && !cal; i++) {
    cal = mpuCalibrate();
    if (!cal && !waitMs(600)) return false;
  }
  headingRef = yawDeg;
#endif
  unsigned long used = millis() - w0;
  if (waitTotalMs > used) {
    Serial.print(F("Waiting "));
    Serial.print((waitTotalMs - used) / 1000);
    Serial.println(F(" s..."));
    if (!waitMs(waitTotalMs - used)) return false;
  }
  return true;
}

// look from the scan pose at this base; small arm sweep if not seen
static bool lookAt(int base, int &x, int &y) {
  static const int8_t OFF[5] = {0, 12, -12, 24, -24};
  for (byte k = 0; k < 5; k++) {
    if (k > 0) {
      Serial.print(F("   not seen - arm sweep "));
      Serial.println(OFF[k]);
    }
    if (!armScanAt(base + OFF[k])) return false;
    if (!waitMs(SETTLE_MS)) return false;
    if (lookForSample(x, y, LOOK_MS)) return true;
    if (missionAborted) return false;
  }
  return false;
}

// colour only (no pick). Robot must be at the slot.
bool lookSlot(byte s, PatientColor &c) {
  SlotData &sd = st.slot[s];
  Pose3 p = sd.ok ? sd.pick : slotDefaultPose(s);
  setAim(sd.hasPix ? sd.px : IMAGE_CENTER_X, sd.hasPix ? sd.py : IMAGE_CENTER_Y);
  int x, y;
  c = lookAt(p.b, x, y) ? lastColor : COL_NONE;
  return !missionAborted;
}

// ---- the 3 ways to move the patient in the picture (learned in "teach N") ----
#define AX_BASE    0     // arm base   (units)
#define AX_STRAFE  1     // wheels     (mm, + = LEFT)
#define AX_FWD     2     // wheels     (mm, + = FORWARD)

static float fineMM[3] = {0, 0, 0};   // wheel correction done for this patient ([AX_STRAFE], [AX_FWD])

static const __FlashStringHelper *axName(byte ax) {
  if (ax == AX_BASE) return F("arm base");
  if (ax == AX_STRAFE) return F("wheels LEFT/RIGHT");
  return F("wheels FORWARD/BACK");
}

// picture shift (px) for 1 unit of this direction. false = not calibrated / not allowed
static bool axisVec(byte s, byte ax, bool wheels, float &vx, float &vy) {
  if (ax == AX_BASE) {
    if (!st.baseCal) return false;
    vx = st.bx; vy = st.by;
  } else if (ax == AX_STRAFE) {
    if (!wheels || !st.strafeCal[s]) return false;
    vx = st.sx[s]; vy = st.sy[s];
  } else {
    if (!wheels || !st.fwdCal[s]) return false;
    vx = st.fwx[s]; vy = st.fwy[s];
  }
  return (fabs(vx) + fabs(vy)) > 0.05;
}

// Choose 2 directions that move the picture in DIFFERENT ways (so every error can be fixed).
// Prefer pairs with the arm base (finest steps). false = no good pair.
static bool choosePair(byte s, bool wheels, byte &a1, byte &a2) {
  static const byte PAIRS[3][2] = {{AX_BASE, AX_STRAFE}, {AX_BASE, AX_FWD}, {AX_STRAFE, AX_FWD}};
  float bestBase = 0, bestAll = 0;
  int iBase = -1, iAll = -1;
  for (byte i = 0; i < 3; i++) {
    float ux, uy, vx, vy;
    if (!axisVec(s, PAIRS[i][0], wheels, ux, uy) || !axisVec(s, PAIRS[i][1], wheels, vx, vy)) continue;
    float sn = fabs(ux * vy - uy * vx) / (sqrt(ux * ux + uy * uy) * sqrt(vx * vx + vy * vy));
    if (PAIRS[i][0] == AX_BASE && sn > bestBase) { bestBase = sn; iBase = i; }
    if (sn > bestAll) { bestAll = sn; iAll = i; }
  }
  int pick = -1;
  if (iBase >= 0 && bestBase >= FINE_MIN_SIN) pick = iBase;
  else if (iAll >= 0 && bestAll >= FINE_MIN_SIN / 2) pick = iAll;
  if (pick < 0) return false;
  a1 = PAIRS[pick][0];
  a2 = PAIRS[pick][1];
  return true;
}

// pixels per mm on the floor (average of the wheel calibrations), 0 = unknown
static float pxPerMm(byte s) {
  float sum = 0;
  byte n = 0;
  if (st.strafeCal[s]) { sum += sqrt(st.sx[s] * st.sx[s] + st.sy[s] * st.sy[s]); n++; }
  if (st.fwdCal[s])    { sum += sqrt(st.fwx[s] * st.fwx[s] + st.fwy[s] * st.fwy[s]); n++; }
  return n ? sum / n : 0;
}

// move the robot a few mm (+ = LEFT / FORWARD) and keep the map right.
// No heading fix here (used while the claw is DOWN - turning would drag it sideways).
static bool wheelMoveNoFix(byte ax, float mm) {
  MoveType t = (ax == AX_STRAFE) ? (mm > 0 ? STRAFE_LEFT : STRAFE_RIGHT)
                                 : (mm > 0 ? FORWARD : BACKWARD);
  if (!smallMove(t, fabs(mm))) return false;
  if (ax == AX_STRAFE) navY += mm;   // so the way back to H CENTER stays right
  else                 navX += mm;
  return true;
}

static bool wheelMove(byte ax, float mm) {
  if (!wheelMoveNoFix(ax, mm)) return false;
  return fixHeading();
}

// Bring the CENTRE of the patient box onto the target pixel:
//   target = pixel recorded in teach  (or the picture middle if ALIGN_TO_IMAGE_CENTER 1)
//   uses the 2 directions from choosePair (arm base / wheels left-right / wheels forward-back)
//   stops as soon as a move does not make the error smaller
// v18d: tighter aim on the slots listed in TIGHT_SLOTS (Config.h)
static bool slotTight(byte s) {
  static const byte tl[] = TIGHT_SLOTS;
  for (byte i = 0; i < sizeof(tl); i++) if (tl[i] == s + 1) return true;
  return false;
}

// v18g: slots listed in SPECIAL_SLOTS (Config.h)
static bool slotSpecial(byte s) {
  static const byte sl[] = SPECIAL_SLOTS;
  for (byte i = 0; i < sizeof(sl); i++) if (sl[i] == s + 1) return true;
  return false;
}

static bool fineCorrect(byte s, int base0, int &x, int &y) {
#if FINE_ENABLE
  SlotData &sd = st.slot[s];
#if ALIGN_TO_IMAGE_CENTER
  int tx = IMAGE_CENTER_X, ty = IMAGE_CENTER_Y;
#else
  if (!sd.hasPix) return true;
  int tx = sd.px, ty = sd.py;
#endif
  byte a1 = AX_BASE, a2 = AX_BASE;
  bool pair = choosePair(s, FINE_WHEELS, a1, a2);
  if (!pair && !st.baseCal) return true;
  if (!pair) Serial.println(F("   FINE: only ONE direction can be corrected - teach this slot again"));

  float u1x = 0, u1y = 0, u2x = 0, u2y = 0, det = 1;
  if (pair) {
    axisVec(s, a1, true, u1x, u1y);
    axisVec(s, a2, true, u2x, u2y);
    det = u1x * u2y - u1y * u2x;
  }
  float ppm = pxPerMm(s);
  float prevErr = 1e9;

  setAim(tx, ty);
  bool tight = slotTight(s);
  float tolPx = tight ? TIGHT_FINE_TOL_PX : FINE_TOL_PX;
  byte rounds = tight ? TIGHT_FINE_ROUNDS : FINE_ROUNDS;
  for (byte r = 0; r <= rounds; r++) {
    int w, h;
    if (!lookPrecise(x, y, w, h, ALIGN_LOOK_MS)) return !missionAborted;
    float ex = tx - x, ey = ty - y;
    float err = sqrt(ex * ex + ey * ey);
    Serial.print(F("   FINE: error dx="));
    Serial.print(ex, 0);
    Serial.print(F(" dy="));
    Serial.print(ey, 0);
    Serial.print(F(" px"));
    if (ppm > 0.05) {
      Serial.print(F("  (~"));
      Serial.print(err / ppm, 1);
      Serial.print(F(" mm)"));
    }
    Serial.println();
    if (fabs(ex) <= tolPx && fabs(ey) <= tolPx) return true;
    if (err > prevErr - FINE_MIN_PROGRESS_PX) {
      Serial.println(F("   FINE: no progress - stop correcting"));
      return true;
    }
    prevErr = err;
    if (r == rounds) break;

    // split the error into the chosen directions
    float amt[3] = {0, 0, 0};
    if (pair) {
      amt[a1] = (ex * u2y - ey * u2x) / det;
      amt[a2] = (u1x * ey - u1y * ex) / det;
    } else {
      amt[AX_BASE] = (st.bx * ex + st.by * ey) / (st.bx * st.bx + st.by * st.by);
    }

    bool moved = false;
    float nb = constrain(armBase + FINE_GAIN * amt[AX_BASE], base0 - FINE_MAX_UNITS, base0 + FINE_MAX_UNITS);
    if (fabs(nb - armBase) >= 0.5) {
      if (!armScanAt((int)(nb + 0.5))) return false;
      moved = true;
    }
    for (byte ax = AX_STRAFE; ax <= AX_FWD; ax++) {
      float v = FINE_GAIN * amt[ax];
      if (fabs(v) < FINE_WHEEL_MIN_MM) continue;
      float nv = constrain(fineMM[ax] + v, -FINE_MAX_MM, FINE_MAX_MM);
      float dv = nv - fineMM[ax];
      if (fabs(dv) < FINE_WHEEL_MIN_MM) continue;
      if (!wheelMove(ax, dv)) return false;
      fineMM[ax] = nv;
      moved = true;
    }
    if (!moved) return true;     // at the limit - nothing more to do
    if (!waitMs(SETTLE_MS)) return false;
  }
#else
  (void)s; (void)base0; (void)x; (void)y;
#endif
  return true;
}

// step for the search pattern in this direction: PICK_STEP_MM (wheels) or the same
// distance in arm-base units (from the calibration; without it PICK_STEP_BASE)
static float patternStep(byte s, byte ax) {
  if (ax != AX_BASE) return PICK_STEP_MM;
  float ppm = pxPerMm(s);
  if (st.baseCal && ppm > 0.05) {
    float pxPerUnit = sqrt(st.bx * st.bx + st.by * st.by);
    if (pxPerUnit > 0.05) return constrain(PICK_STEP_MM * ppm / pxPerUnit, 1.0, 8.0);
  }
  return PICK_STEP_BASE;
}

// move along one pattern direction (base: absolute target, wheels: relative mm)
static bool patternMove(byte ax, float amount) {
  if (amount == 0) return true;
  if (ax == AX_BASE) return true;          // base offset is applied in the pick pose
  return wheelMove(ax, amount);
}

#if GRIPPER_CLAW && CLAW_HELD_VISIBLE
// "FLOOR OR CLAW?" - the camera saw a patient near the pick spot after the lift.
// Swing the arm base so a patient ON THE FLOOR must slide ~HELD_SHIFT_PX in the picture
// (direction known from the camera base cal). A patient IN THE CLAW moves with the camera.
//   1 = in the claw (seen at the same pixel)
//   0 = on the floor (seen exactly where a floor patient must be now)
//  -1 = not sure (camera saw neither)
static int clawHeldTest(float base0, int vx, int vy) {
  int b0 = (int)(base0 + 0.5);
  float n = sqrt(st.bx * st.bx + st.by * st.by);
  bool predict = st.baseCal && n > 0.05;
  int units = predict ? (int)constrain(HELD_SHIFT_PX / n + 0.5, 4, HELD_TEST_UNITS) : HELD_TEST_UNITS;

  // swing direction that stays inside the servo range and keeps the floor patient in the picture
  int du = 0;
  for (int sg = 1; sg >= -1 && du == 0; sg -= 2) {
    int tb = b0 + sg * units;
    if (tb < BASE_MIN + 5 || tb > BASE_MAX - 5) continue;
    if (predict) {
      float px = vx + st.bx * sg * units, py = vy + st.by * sg * units;
      if (px < 10 || px > IMAGE_W - 10 || py < 10 || py > IMAGE_H - 10) continue;
    }
    du = sg * units;
  }
  if (du == 0) {                       // floor place would leave the picture: swing anyway
    predict = false;
    du = (b0 + units <= BASE_MAX - 5) ? units : -units;
  }
  int fx = (int)(vx + st.bx * du), fy = (int)(vy + st.by * du);

  Serial.print(F("CHECK: floor or claw? swing the arm base "));
  Serial.print(du);
  Serial.print(F(" units"));
  if (predict) {
    Serial.print(F(" - a FLOOR patient must move to x="));
    Serial.print(fx);
    Serial.print(F(" y="));
    Serial.print(fy);
  }
  Serial.println();
  if (!armScanAt(b0 + du)) return -1;
  if (!waitMs(VERIFY_SETTLE_MS)) return -1;

  long rs = (long)HELD_SAME_PX * HELD_SAME_PX, rf = (long)HELD_FLOOR_PX * HELD_FLOOR_PX;
  byte answered = 0, sameHits = 0, floorHits = 0;
  // v14: throw away OLD pictures (taken before the swing)
  for (byte k = 0, n = 0; k < VERIFY_SKIP_FRAMES && n < 10; n++) {
    if (abortRequested()) return -1;
    if (gv2Invoke() == 0) k++;
  }
  unsigned long t0 = millis();
  while (answered < VERIFY_CLEAR_FRAMES + 2 && millis() - t0 < VERIFY_MS) {
    if (abortRequested()) return -1;
    if (gv2Invoke() != 0) continue;
    answered++;
    bool same = false, onFloor = false;
    for (int i = 0; i < boxCount; i++) {
      if (!isPatientClass(boxes[i].target) || boxes[i].score < VERIFY_MIN_SCORE) continue;
      long dx = boxes[i].x - vx, dy = boxes[i].y - vy;
      if (dx * dx + dy * dy <= rs) same = true;
      if (predict) {
        dx = boxes[i].x - fx; dy = boxes[i].y - fy;
        if (dx * dx + dy * dy <= rf) onFloor = true;
      }
    }
    if (same) sameHits++;
    if (onFloor) floorHits++;
  }
  if (!armScanAt(b0)) return -1;

  Serial.print(F("CHECK: "));
  Serial.print(answered);
  Serial.print(F(" pictures: same place "));
  Serial.print(sameHits);
  Serial.print(F("x, floor place "));
  Serial.print(floorHits);
  Serial.println('x');
  // the patient at the SAME pixel is the direct proof of "held"
  // (a box at the floor place could be a neighbour)
  if (sameHits >= HELD_MIN_HITS) {
    Serial.println(F("CHECK: patient moved WITH the camera -> it is IN THE CLAW"));
    return 1;
  }
  if (floorHits >= HELD_MIN_HITS) {
    Serial.println(F("CHECK: patient slid like the floor -> it is ON THE FLOOR"));
    return 0;
  }
  Serial.println(F("CHECK: camera saw neither -> NOT SURE"));
  return -1;
}
#endif

// ======================================================
// SLOT POSITION FIX (v18): arm at the taught scan pose of slot s. The patient must appear
// at the pixel recorded in "teach N". Move the WHEELS (left/right + forward/back) until it
// does -> the robot stands exactly where it stood in "teach N" -> SET navX / navY.
// false = stopped. A fix that is not possible is skipped (the run goes on).
// ======================================================
static bool slotPosFix(byte s, int base) {
#if POSFIX_ENABLE
  SlotData &sd = st.slot[s];
  if (!sd.hasPix || !st.strafeCal[s] || !st.fwdCal[s]) {
    Serial.println(F("POS FIX: slot has no wheel calibration (teach N again) - skipped"));
    return true;
  }
  float lx = st.sx[s], ly = st.sy[s], fx = st.fwx[s], fy = st.fwy[s];
  float det = lx * fy - ly * fx;
  if (fabs(det) < 0.05) {
    Serial.println(F("POS FIX: calibration of this slot is bad (teach N again) - skipped"));
    return true;
  }
  if (!armScanAt(base)) return false;
  if (!waitMs(SETTLE_MS)) return false;
  setAim(sd.px, sd.py);

  float movedL = 0, movedF = 0;
  bool fixed = false;
  bool tight = slotTight(s);
  float tolMM = tight ? TIGHT_POSFIX_TOL_MM : POSFIX_TOL_MM;
  byte rounds = tight ? TIGHT_POSFIX_ROUNDS : POSFIX_ROUNDS;
  if (tight) Serial.println(F("POS FIX: TIGHT slot - smaller tolerance"));
  for (byte r = 0; r <= rounds; r++) {
    int x, y, w, h;
    if (!lookPrecise(x, y, w, h, ALIGN_LOOK_MS)) {
      if (missionAborted) return false;
      Serial.println(F("POS FIX: patient not seen clearly - no fix"));
      break;
    }
    float ex = sd.px - x, ey = sd.py - y;
    float L = (ex * fy - ey * fx) / det;          // mm robot must go LEFT
    float Fw = (lx * ey - ly * ex) / det;         // mm robot must go FORWARD
    Serial.print(F("POS FIX: robot is "));
    Serial.print(-L, 1);
    Serial.print(F(" mm left, "));
    Serial.print(-Fw, 1);
    Serial.println(F(" mm forward of the taught place"));
    if (fabs(L) <= tolMM && fabs(Fw) <= tolMM) { fixed = true; break; }
    if (r == 0 && sqrt(L * L + Fw * Fw) > POSFIX_MAX_MM) {
      Serial.println(F("POS FIX: error too big - not sure, no fix"));
      break;
    }
    if (r == rounds) break;
    L *= POSFIX_GAIN;
    Fw *= POSFIX_GAIN;
    bool mv = false;
    if (fabs(L) >= POSFIX_MIN_MOVE) {
      if (!smallMove(L > 0 ? STRAFE_LEFT : STRAFE_RIGHT, fabs(L))) return false;
      movedL += L; mv = true;
    }
    if (fabs(Fw) >= POSFIX_MIN_MOVE) {
      if (!smallMove(Fw > 0 ? FORWARD : BACKWARD, fabs(Fw))) return false;
      movedF += Fw; mv = true;
    }
    if (!fixHeading()) return false;
    if (!mv) { fixed = true; break; }
    if (!waitMs(SETTLE_MS)) return false;
  }

  if (fixed) {
    float sx, sy;
    slotStop(s, sx, sy);
    Serial.print(F("POS FIX: OK - map reset to the slot ("));
    Serial.print(sx, 0);
    Serial.print(F(", "));
    Serial.print(sy, 0);
    Serial.print(F(")   slip removed: "));
    Serial.print(-movedF, 1);
    Serial.print(F(" / "));
    Serial.print(-movedL, 1);
    Serial.println(F(" mm"));
    navX = sx;                     // robot = exactly the taught slot stop
    navY = sy;
  } else {
    navX += movedF;                // not sure: only book the wheel moves
    navY += movedL;
  }
#else
  (void)s; (void)base;
#endif
  return true;
}

// FIND + PICK + CHECK. c = colour (COL_NONE = nothing picked). false = stopped
bool pickSlot(byte s, PatientColor &c) {
  c = COL_NONE;
  fineMM[AX_STRAFE] = 0;
  fineMM[AX_FWD] = 0;
  SlotData &sd = st.slot[s];
  Pose3 p = sd.ok ? sd.pick : slotDefaultPose(s);
  if (!sd.ok) Serial.println(F("!! slot NOT taught - using the default pose (send: teach N)"));

  int aimX = sd.hasPix ? sd.px : IMAGE_CENTER_X;
  int aimY = sd.hasPix ? sd.py : IMAGE_CENTER_Y;
  setAim(aimX, aimY);

  int x, y;
  if (!lookAt(p.b, x, y)) {
    if (missionAborted) return false;
    Serial.println(F("NO PATIENT on this slot"));
    return true;
  }
  PatientColor seen = lastColor;
  Serial.print(F("COLOUR: "));
  Serial.println(colorName(seen));

  // v18: put the robot back EXACTLY on the taught slot place + reset the map (removes slip)
  if (!slotPosFix(s, p.b)) return false;
  setAim(aimX, aimY);

  // SEARCH PATTERN around the aimed point (brute force), PICK_STEP_MM apart.
  //   column 1 / column 2 = the 2 directions from choosePair (usually arm base + the wheel
  //   direction that moves the picture most differently). Without calibration: base + strafe.
  //
  //        7      2      5            1 = aimed point
  //        3      1      4            2..8 = PICK_STEP_MM around it
  //        8      9*     6            (* 9 = only with PICK_MAX_TRIES 9)
  static const int8_t PAT[9][2] = {
    { 0,  0},   // 1 aimed point
    { 0,  1},   // 2 strafe left
    {-1,  0},   // 3 base -
    { 1,  0},   // 4 base +
    { 1,  1},   // 5 diagonal
    { 1, -1},   // 6 diagonal
    {-1,  1},   // 7 diagonal
    {-1, -1},   // 8 diagonal
    { 0, -1},   // 9 strafe right
  };
  byte tries = PICK_MAX_TRIES > 9 ? 9 : PICK_MAX_TRIES;
  byte pa1 = AX_BASE, pa2 = AX_STRAFE;
  if (!choosePair(s, PICK_SHIFT_WHEELS, pa1, pa2)) { pa1 = AX_BASE; pa2 = AX_STRAFE; }
  if (!PICK_SHIFT_WHEELS) pa2 = AX_BASE;          // base only
  float step1 = patternStep(s, pa1), step2 = patternStep(s, pa2);
  Serial.print(F("SEARCH PATTERN: "));
  Serial.print(axName(pa1));
  Serial.print(F(" + "));
  Serial.println(pa2 == pa1 ? F("(nothing)") : axName(pa2));
  bool picked = false;

  for (byte a = 0; a < tries && !picked; a++) {
    // aim at the patient again first (the last try may have pushed it)
    if (!fineCorrect(s, p.b, x, y)) return false;
    float base0 = (FINE_ENABLE && (sd.hasPix || ALIGN_TO_IMAGE_CENTER) && st.baseCal) ? armBase : p.b;

    // offsets for this try: arm base (units) and wheels (mm, which direction = pa1 / pa2)
    float off1 = PAT[a][0] * step1, off2 = (pa2 == pa1) ? 0 : PAT[a][1] * step2;
    float base = base0 + (pa1 == AX_BASE ? off1 : 0) + (pa2 == AX_BASE ? off2 : 0);
    byte wAx[2] = {pa1, pa2};
    float wMM[2] = {pa1 == AX_BASE ? 0 : off1, (pa2 == AX_BASE || pa2 == pa1) ? 0 : off2};
    float pb = p.bo + (a > 0 ? PICK_RETRY_PRESS : 0);    // retries press a little lower
    float pt = p.t;
    if (seen == COL_GREEN) { pb += GREEN_PRESS_BOTTOM; pt += GREEN_PRESS_TOP; }   // v18d

    Serial.print(F("===> PICK TRY "));
    Serial.print(a + 1);
    Serial.print('/');
    Serial.print(tries);
    Serial.print(F("   base "));
    Serial.print(base - base0, 1);
    Serial.print(F(" units"));
    for (byte k = 0; k < 2; k++) {
      if (wMM[k] == 0) continue;
      Serial.print(F("   "));
      Serial.print(axName(wAx[k]));
      Serial.print(' ');
      Serial.print(wMM[k], 0);
      Serial.print(F(" mm"));
    }
    Serial.println();

    for (byte k = 0; k < 2; k++)
      if (!patternMove(wAx[k], wMM[k])) return false;

    // v18e SAFETY: never put the arm down when the robot or the arm base is turned wrong
    {
      bool ok = true;
#if USE_MPU
      if (mpuOk) {
        float he = headingRef - yawDeg;
        if (fabs(he) > PICK_MAX_HEADING_DEG) {
          Serial.print(F("   SAFETY: robot turned "));
          Serial.print(he, 1);
          Serial.println(F(" deg - fixing heading first"));
          if (!fixHeading()) return false;
          he = headingRef - yawDeg;
          if (fabs(he) > PICK_MAX_HEADING_DEG) ok = false;
        }
      }
#endif
      float bd = base - p.b;
      Serial.print(F("   arm base off taught: "));
      Serial.print(bd, 1);
      Serial.println(F(" units"));
      if (fabs(bd) > PICK_MAX_BASE_DEV) ok = false;
      if (!ok) {
        Serial.println(F("   SAFETY: rotation is off -> arm NOT going down, re-aim and try again"));
        if (!armScanAt(p.b)) return false;
        if (!waitMs(SETTLE_MS)) return false;
        x = aimX; y = aimY;
        continue;
      }
    }

#if GRIPPER_CLAW && CLAW_APPROACH
    // "in front" for the claw: along the arm away from the robot (right slots: arm points RIGHT)
    byte apAx = (CLAW_APPROACH_AXIS == 2) ? AX_FWD : AX_STRAFE;
    float apDir = CLAW_APPROACH_SIGN;
    if (apAx == AX_STRAFE && slotIsRight(s)) apDir = -apDir;   // + = LEFT, so RIGHT = minus
    // v18f: special slot = own back-off / push, and later tries aim further AHEAD
    float clawBack = CLAW_BACKOFF_MM, clawPush = CLAW_PUSH_MM, ahead = 0;
    byte aheadAx = (SPECIAL_AHEAD_AXIS == 2) ? AX_FWD : apAx;
    float aheadDir = (SPECIAL_AHEAD_AXIS == 2) ? 1.0 : apDir;
    if (slotSpecial(s)) {
      clawBack = SPECIAL_BACKOFF_MM;
      clawPush = SPECIAL_PUSH_MM;
      if (a >= SPECIAL_AHEAD_AFTER) ahead = SPECIAL_AHEAD_MM;
    }
    if (ahead != 0) {
      Serial.print(F("   SPECIAL SLOT: aiming "));
      Serial.print(ahead, 0);
      Serial.println(F(" mm AHEAD of the taught spot"));
      if (!wheelMove(aheadAx, aheadDir * ahead)) return false;
    }
    // v18i: small FORWARD offset for every pick (undone after the try)
    if (PICK_FWD_OFFSET_MM != 0) {
      Serial.print(F("   pick offset: forward "));
      Serial.print(PICK_FWD_OFFSET_MM, 1);
      Serial.println(F(" mm"));
      if (!wheelMove(AX_FWD, PICK_FWD_OFFSET_MM)) return false;
    }
#endif
    // v18i: pause before the arm goes down
    if (PICK_PAUSE_MS > 0) {
      Serial.print(F("   pause "));
      Serial.print(PICK_PAUSE_MS / 1000);
      Serial.println(F(" s before the pick..."));
      if (!waitMs(PICK_PAUSE_MS)) return false;
    }
    if (!armOver(base, pb, pt)) return false;
#if GRIPPER_CLAW
    setSuction(false);                              // claw OPEN before going down
    if (!waitMs(CLAW_MOVE_MS)) return false;
#if CLAW_APPROACH
    // 1. land SHORT: move back CLAW_BACKOFF_MM while the arm is still up
    Serial.print(F("   claw approach: land "));
    Serial.print(clawBack, 0);
    Serial.print(F(" mm short, then slide "));
    Serial.print(clawPush, 0);
    Serial.println(F(" mm in front"));
    if (!wheelMove(apAx, -apDir * clawBack)) return false;
#endif
    if (!armDown(base, pb, pt)) return false;       // 2. claw down (open)
#if CLAW_APPROACH
    // 3. slide the open claw onto the patient (arm stays down, no heading turn)
    if (!wheelMoveNoFix(apAx, apDir * clawPush)) return false;
#endif
    setSuction(true);                               // 4. claw CLOSE around the patient
    Serial.println(F("   claw closing on the patient..."));
    if (!waitMs(CLAW_MOVE_MS + CLAW_HOLD_MS)) return false;
#else
    setSuction(true);
    if (!armDown(base, pb, pt)) return false;
    Serial.println(F("   nozzle on the patient - sucking..."));
    if (!waitMs(PICK_DWELL_MS)) return false;
#endif
    float hb = pb + HOVER_FRACTION * (st.scan.bo - pb);
    float ht = pt + HOVER_FRACTION * (st.scan.t - pt);
    if (!armMoveTo((int)(base + 0.5), (int)(hb + 0.5), (int)(ht + 0.5), PICK_LIFT_MS)) return false;
#if GRIPPER_CLAW && CLAW_APPROACH
    // robot is now (PUSH - BACKOFF) mm past the aimed point: straighten, then move back
    if (!fixHeading()) return false;
    if (!wheelMove(apAx, -apDir * (clawPush - clawBack))) return false;
#endif

    // back to the aimed point, so the camera check looks at the same place
    if (!armScanAt((int)(base0 + 0.5))) return false;
    for (byte k = 0; k < 2; k++)
      if (!patternMove(wAx[k], -wMM[k])) return false;
#if GRIPPER_CLAW && CLAW_APPROACH
    if (ahead != 0 && !wheelMove(aheadAx, -aheadDir * ahead)) return false;   // v18f: undo AHEAD
    if (PICK_FWD_OFFSET_MM != 0 && !wheelMove(AX_FWD, -PICK_FWD_OFFSET_MM)) return false;  // v18i: undo
#endif

    // pick check switched off (command: verify 0) -> trust the claw, one try only
    if (!st.pickVerify) {
      Serial.println(F("CHECK: off (verify 0) -> PICK assumed OK"));
      picked = true;
      break;
    }
    if (!waitMs(VERIFY_SETTLE_MS)) return false;

    // search circle: at least VERIFY_RADIUS_PX, bigger for a big patient (w / h)
    int size = lastBoxW > lastBoxH ? lastBoxW : lastBoxH;
    int radius = (int)(VERIFY_RADIUS_SIZE * size);
    if (radius < VERIFY_RADIUS_PX) radius = VERIFY_RADIUS_PX;

    int vx = x, vy = y;
    int r = pickCheck(x, y, radius, vx, vy);
    if (missionAborted) return false;
    bool lowRelease = false;       // not sure what is in the claw -> open it LOW over the spot
#if GRIPPER_CLAW
    if (r == -1) {                 // camera did not answer enough: one more look, no swing
      Serial.println(F("CHECK: not sure - looking once more"));
      r = pickCheck(x, y, radius, vx, vy);
      if (missionAborted) return false;
      if (r == -1) lowRelease = true;
    }
#if CLAW_HELD_VISIBLE
    if (r == 0) {
      // the camera may see the patient HELD in the claw -> floor or claw?
      int h = clawHeldTest(base0, vx, vy);
      if (missionAborted) return false;
      if (h == 1) r = 1;
      else {
        if (h == -1) lowRelease = true;
        if (!waitMs(VERIFY_SETTLE_MS)) return false;
        // where is it on the floor now (for the next try)?
        setAim(vx, vy);
        int fx = vx, fy = vy;
        if (lookForSample(fx, fy, LOOK_MS)) { vx = fx; vy = fy; }
        if (missionAborted) return false;
      }
    }
#else
    // camera can NOT see a held patient: a patient at the pick spot = still ON THE FLOOR
    if (r == 0) Serial.println(F("CHECK: patient still on the floor -> NOT picked"));
#endif
#endif
    if (r == 1) {
      Serial.println(F("CHECK: VERIFIED - pick spot is EMPTY -> PICK OK, going to drop"));
      picked = true;
    } else {
      Serial.println(r == 0 ? F("CHECK: -> NOT picked, again")
                            : F("CHECK: no proof -> treat as NOT picked, again"));
#if GRIPPER_CLAW
      if (lowRelease) {
        // maybe the claw DOES hold it: open it just above the spot, so it lands
        // back on its dot (and the next try picks it up again) - no drop from high up
        Serial.println(F("   not sure what is in the claw -> open it LOW over the spot"));
        if (!armOver(base0, p.bo, p.t)) return false;
        setSuction(false);
        if (!waitMs(CLAW_MOVE_MS)) return false;
        if (!armScanAt((int)(base0 + 0.5))) return false;
      } else
#endif
      setSuction(false);
      if (!waitMs(400)) return false;
      x = vx;                    // aim at where it is now (it may have been pushed)
      y = vy;
      setAim(x, y);
    }
  }

  if (!picked) {
    Serial.println(F("!!! PICK FAILED - going on"));
    setSuction(false);
    return true;
  }
  c = seen;
  return true;
}

// drive to the zone and drop. Ends in INITIAL.
bool placeInZone(Zone z, byte k, byte n) {
  if (!navToDrop(z, k)) return false;          // v18h: stops DROP_BACK_..._MM short
  if (!mgate(F("PLACE"))) return false;
  Pose3 p = st.zone[z].place;
  if (!st.zone[z].ok) Serial.println(F("!! zone NOT taught - default place pose (send: tz ...)"));
  if (!armOver(p.b, p.bo, p.t)) return false;
  if (!armDown(p.b, p.bo, p.t)) return false;

  // RELEASE: pump OFF (sent twice), arm stays down, shake a little, wait
  setSuction(false);
  delay(30);
  setSuction(false);
  if (!waitMs(300)) return false;
  for (byte w = 0; w < RELEASE_WIGGLE; w++) {
    if (!armMoveTo(p.b + RELEASE_WIGGLE_UNITS, p.bo, p.t, 120)) return false;
    if (!armMoveTo(p.b - RELEASE_WIGGLE_UNITS, p.bo, p.t, 120)) return false;
  }
  if (!armMoveTo(p.b, p.bo, p.t, 120)) return false;
  if (!waitMs(RELEASE_WAIT_MS)) return false;

  // slow lift so the patient stays on the floor
  {
    float hb = p.bo + HOVER_FRACTION * (st.scan.bo - p.bo);
    float ht = p.t  + HOVER_FRACTION * (st.scan.t  - p.t);
    if (!armMoveTo(p.b, (int)(hb + 0.5), (int)(ht + 0.5), PLACE_LIFT_MS)) return false;
  }
  if (!armToInitial()) return false;
  Serial.print(F("PLACED patient "));
  Serial.print(n);
  Serial.print(F(" in "));
  Serial.println(zoneName(z));
  return true;
}

// ======================================================
// CAPTURE FIX (v15): robot is at slot s (by wheel counts + gyro). Look with the captured
// arm pose, move the WHEELS until the patient is at the captured pixel, then SET the map:
// the robot now stands exactly where it stood when "capture N" was taught = the slot stop.
// false = stopped (x / STOP). A failed fix is NOT an error: the run goes on without it.
// ======================================================
// result of the last capture fix (printed at the END of "capcheck")
//  0 not run  1 no capture saved  2 bad calibration  3 patient not seen  4 size differs
//  5 error too big  6 FIXED  7 moved but not finished
byte capResult = 0;
float capLastL = 0, capLastF = 0;     // last measured error (mm, + = robot must go left / forward)

bool captureFix(byte s) {
  capResult = 0;
#if CAPTURE_ENABLE
  CapData &cd = st.cap[s];
  if (st.capMagic != CAP_MAGIC || !cd.ok) {
    Serial.print(F("--- CAPTURE FIX: slot "));
    Serial.print(s + 1);
    Serial.println(F(" has NO capture saved (type: capture N) - skipped"));
    capResult = 1;
    return true;
  }
  float det = cd.lx * cd.fy - cd.ly * cd.fx;
  if (fabs(det) < 0.05) {
    Serial.println(F("--- CAPTURE FIX: calibration of this slot is bad - skipped (type: capture N again)"));
    capResult = 2;
    return true;
  }

  Serial.print(F("--- CAPTURE FIX (camera position check): arm to base="));
  Serial.print(cd.pose.b);
  Serial.print(F(" bottom="));
  Serial.print(cd.pose.bo);
  Serial.print(F(" top="));
  Serial.println(cd.pose.t);
  if (!armToPose(cd.pose, ARM_MOVE_MS)) return false;
  if (!waitMs(SETTLE_MS)) return false;
  setAim(cd.x, cd.y);

  float movedL = 0, movedF = 0;      // what the wheels did here (mm, + = left / forward)
  bool fixed = false;
  for (byte r = 0; r <= CAP_ROUNDS; r++) {
    int x, y, w, h;
    if (!lookPrecise(x, y, w, h, ALIGN_LOOK_MS)) {
      if (missionAborted) return false;
      Serial.println(F("   capture: patient not seen - no fix"));
      capResult = 3;
      break;
    }
    if (abs(w - cd.w) > CAP_SIZE_TOL * cd.w || abs(h - cd.h) > CAP_SIZE_TOL * cd.h) {
      Serial.print(F("   capture: size "));
      Serial.print(w); Serial.print('x'); Serial.print(h);
      Serial.print(F(" but captured "));
      Serial.print(cd.w); Serial.print('x'); Serial.print(cd.h);
      Serial.println(F(" - not the same view, no fix"));
      capResult = 4;
      break;
    }
    float ex = cd.x - x, ey = cd.y - y;          // picture error (px)
    // ex = L*lx + F*fx ;  ey = L*ly + F*fy   ->  L = mm LEFT, F = mm FORWARD
    float L = (ex * cd.fy - ey * cd.fx) / det;
    float Fw = (cd.lx * ey - cd.ly * ex) / det;
    capLastL = L;
    capLastF = Fw;
    Serial.print(F("   capture: picture error dx="));
    Serial.print(ex, 0);
    Serial.print(F(" dy="));
    Serial.print(ey, 0);
    Serial.print(F(" px  = robot off by "));
    Serial.print(-L, 1);
    Serial.print(F(" mm left, "));
    Serial.print(-Fw, 1);
    Serial.println(F(" mm forward"));
    if (fabs(L) <= CAP_TOL_MM && fabs(Fw) <= CAP_TOL_MM) { fixed = true; break; }
    if (r == CAP_ROUNDS) break;
    if (r == 0 && sqrt(L * L + Fw * Fw) > CAP_MAX_MM) {
      Serial.println(F("   capture: error too big - something is wrong, no fix"));
      capResult = 5;
      break;
    }
    L *= CAP_GAIN;
    Fw *= CAP_GAIN;
    bool mv = false;
    if (fabs(L) >= CAP_MIN_MOVE_MM) {
      if (!smallMove(L > 0 ? STRAFE_LEFT : STRAFE_RIGHT, fabs(L))) return false;
      movedL += L;
      mv = true;
    }
    if (fabs(Fw) >= CAP_MIN_MOVE_MM) {
      if (!smallMove(Fw > 0 ? FORWARD : BACKWARD, fabs(Fw))) return false;
      movedF += Fw;
      mv = true;
    }
    if (!fixHeading()) return false;
    if (!mv) { fixed = true; break; }           // remaining error smaller than one move
    if (!waitMs(SETTLE_MS)) return false;
  }

  if (fixed) capResult = 6;
  else if (capResult == 0) capResult = 7;
  if (fixed) {
    // the robot now stands where it stood in "capture N" = the slot stop -> SET the map
    float sx, sy;
    slotStop(s, sx, sy);
    Serial.print(F("   NAV FIX: map said ("));
    Serial.print(navX + movedF, 0);
    Serial.print(F(", "));
    Serial.print(navY + movedL, 0);
    Serial.print(F(")  ->  real ("));
    Serial.print(sx, 0);
    Serial.print(F(", "));
    Serial.print(sy, 0);
    Serial.print(F(")   drift corrected: "));
    Serial.print(sx - (navX + movedF), 1);
    Serial.print(F(" / "));
    Serial.print(sy - (navY + movedL), 1);
    Serial.println(F(" mm"));
    navX = sx;
    navY = sy;
  } else {
    // not sure where we are: just book the wheel moves that were done
    navX += movedF;
    navY += movedL;
  }
#else
  (void)s;
#endif
  return true;
}

// FROM H CENTER: slot -> pick -> H CENTER -> zone -> place -> H CENTER
bool processSlot(byte s) {
  bool right = slotIsRight(s);
  Serial.println();
  Serial.print(F("########## PATIENT SLOT "));
  Serial.print(s + 1);
  Serial.print(F("   [t="));
  Serial.print((millis() - missionT0) / 1000);
  Serial.println(F(" s] ##########"));

  if (!armToInitial()) return false;
  if (!navToSlot(s)) return false;
  if (!captureFix(s)) return false;           // v15: camera position fix (if captured)
  if (!mgate(F("PICK"))) return false;

  PatientColor c;
  if (!pickSlot(s, c)) return false;

  if (c == COL_NONE) {
    if (!armToInitial()) return false;
    return navReturnNode();
  }
  if (!navReturnNode()) return false;

  Zone z = zoneFor(c, right);
  Serial.print(colorName(c));
  Serial.print(F(" -> "));
  Serial.println(zoneName(z));
  if (!placeInZone(z, dropCount[z], s + 1)) return false;
  dropCount[z]++;
  patientsDone++;
  return navReturnNode();
}

// ======================================================
// S-RUN (command: s): the competition run, but it STOPS after every job
// (H CENTER reached, each kit, each patient) and waits until you type s again.
// x = stop.   Normal "start" / START button: no stops.
// ======================================================
bool sRun = false;

bool sPause(const __FlashStringHelper *done) {
  if (!sRun) return true;
  stopMotors();
  Serial.println();
  Serial.print(F(">>>>> S-STEP DONE: "));
  Serial.println(done);
  Serial.println(F(">>>>> type  s  = next job      x = stop"));
  char line[16];
  while (true) {
    if (!readLine(line, sizeof(line), true)) return false;     // STOP button
    char c = line[0] | 0x20;
    if (c == 's') { Serial.println(F(">>>>> next...")); return true; }
    if (c == 'x') { missionAborted = true; stopMotors(); return false; }
    Serial.println(F("   type s (next) or x (stop)"));
  }
}

bool deliverKits() {
  if (!mgate(F("=== KITS ==="))) return false;
  if (!armToInitial()) return false;
  if (!navToZone(ZONE_HOSPITAL, 0)) return false;
  if (!kitRelease(ZONE_HOSPITAL)) return false;
  if (!sPause(F("kit delivered to HOSPITAL"))) return false;
  if (st.kitsPCC) {
    if (!navToZone(ZONE_PCC1, 0)) return false;
    if (!kitRelease(ZONE_PCC1)) return false;
    if (!sPause(F("kit delivered to PCC1"))) return false;
    if (!navToZone(ZONE_PCC2, 0)) return false;
    if (!kitRelease(ZONE_PCC2)) return false;
    if (!sPause(F("kit delivered to PCC2"))) return false;
  }
  if (!navReturnNode()) return false;
  return sPause(F("kits done, back on H CENTER"));
}

bool runFromNode(bool kits) {
  if (kits && !deliverKits()) return false;
  for (byte s = 0; s < SLOT_COUNT; s++) {
    if (!processSlot(s)) return false;
    if (s + 1 < SLOT_COUNT && !sPause(F("patient done, back on H CENTER"))) return false;
  }
  setSuction(false);
  armToInitial();
  return true;
}

// COMPETITION: start zone -> wait -> H CENTER -> kits -> 12 patients
bool runMission() {
  missionReset();
  Serial.println(F("############ MISSION 2 START ############"));
  setSuction(false);
  kitsHoldAll();

  if (!armToInitial()) return false;
  if (!gyroStart(START_DELAY_MS )) return false;   // v18i: 5 s + 10 s
  
  missionT0 = millis();
  if (!navStartToNode()) return false;
  if (!sPause(F("start zone -> H CENTER"))) return false;
  if (!runFromNode(RUN_KITS)) return false;
  Serial.println(F("############ MISSION 2 COMPLETE ############"));
  printSummary();
  return true;
}

// KIT TEST (command: startkits): exactly the start of the competition run, then STOP.
//   start zone -> H CENTER (turn right 90) -> HOSPITAL kit -> PCC1 kit -> PCC2 kit -> H CENTER
bool startKitsOnly() {
  missionReset();
  Serial.println(F("##### KIT TEST: START ZONE -> H CENTER -> HOSPITAL -> PCC1 -> PCC2 -> H CENTER #####"));
  setSuction(false);
  kitsHoldAll();
  if (!armToInitial()) return false;
  if (!gyroStart(START_DELAY_MS )) return false;   // v18i: 5 s + 10 s
  missionT0 = millis();
  if (!navStartToNode()) return false;
  if (!deliverKits()) return false;
  armToInitial();
  Serial.println(F("##### KITS DONE - robot stopped on H CENTER #####"));
  return true;
}

// MOVEMENT ONLY: H CENTER -> each zone -> H CENTER (no arm, no pick)
bool zoneTour() {
  const Zone order[4] = {ZONE_HOSPITAL, ZONE_PCC1, ZONE_PCC2, ZONE_RECOVERY};
  if (!armToInitial()) return false;
  for (byte i = 0; i < 4; i++) {
    if (!mgate(zoneName(order[i]))) return false;
    if (!navToZone(order[i], 0)) return false;
    Serial.print(F("   at "));
    Serial.println(zoneName(order[i]));
    if (!waitMs(1500)) return false;
    if (!navReturnNode()) return false;
    Serial.println(F("   back on H CENTER"));
    if (!waitMs(1000)) return false;
  }
  return true;
}

// MOVEMENT ONLY: H CENTER -> every slot -> H CENTER
bool slotTour() {
  if (!armToInitial()) return false;
  for (byte s = 0; s < SLOT_COUNT; s++) {
    if (!navToSlot(s)) return false;
    if (!waitMs(1000)) return false;
    if (!navReturnNode()) return false;
  }
  return true;
}

// colour of every slot, no pick (robot starts on H CENTER)
void lookAll() {
  PatientColor seen[SLOT_COUNT];
  for (byte s = 0; s < SLOT_COUNT; s++) {
    seen[s] = COL_NONE;
    if (!armToInitial() || !navToSlot(s)) return;
    if (!lookSlot(s, seen[s])) return;
    Serial.print(F(">>> slot "));
    Serial.print(s + 1);
    Serial.print(F(": "));
    Serial.println(colorName(seen[s]));
    if (!armToInitial() || !navReturnNode()) return;
  }
  Serial.println(F("===== COLOUR CHECK ====="));
  for (byte s = 0; s < SLOT_COUNT; s++) {
    Serial.print(F("  slot "));
    Serial.print(s + 1);
    Serial.print(F(": "));
    Serial.print(colorName(seen[s]));
    if (seen[s] != COL_NONE) {
      Serial.print(F(" -> "));
      Serial.print(zoneName(zoneFor(seen[s], slotIsRight(s))));
    }
    Serial.println();
  }
}

void printSummary() {
  Serial.print(F("Patients delivered: "));
  Serial.println(patientsDone);
  for (byte z = 0; z < ZONE_COUNT; z++) {
    Serial.print(F("   "));
    Serial.print(zoneName((Zone)z));
    Serial.print(F(": "));
    Serial.println(dropCount[z]);
  }
  Serial.print(F("Time: "));
  Serial.print((millis() - missionT0) / 1000);
  Serial.println(F(" s"));
}