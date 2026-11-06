// ======================================================
// MISSION 1 - BOTICS ALPHA ST68
//
// TABS (Arduino IDE shows them as tabs):
//   Mission1.ino  this file: the mission steps, serial keys, setup
//   Config.h      ALL numbers you tune
//   Movement.ino  wheels (encoders, drive)
//   Arm.ino       arm, suction (PCA9685 ch 5 or Mega pin 7), slab servo
//   Vision.ino    Grove Vision AI V2 camera
//   Pick.ino      sweep-find, arm reach, pick, place, saved settings
//
// START: press the START button (pin 22 to GND) OR send P.
// STOP : press the STOP button (pin 29 to GND) OR send X, any time.
// Serial Monitor: 115200 baud
//   P = START mission      X = STOP (any time)
//
// FIRST-TIME SETUP (once, saved in EEPROM):
//   1. S  -> suction pump must turn ON / OFF (else SUCTION_INVERT in Config.h)
//   2. G  -> guided setup: show ONE spot (nozzle on the sample),
//            the robot then calibrates itself
//   3. K  -> put a sample anywhere in front: find + align + pick test
//   4. O  -> find + pick + place in one go
//
// THE ROBOT ALIGNS BY ITSELF: it looks, turns the arm and drives forward/back,
// round after round, until the sample is under the nozzle, then picks.
//
// Library needed: Adafruit PWM Servo Driver
// Close gv2_detect.py on the PC while this runs.
// ======================================================
#include "Config.h"

bool missionAborted = false;
bool anyKeySeen = false;        // any key since power-up cancels AUTO_START
bool lastButtonState = HIGH;
unsigned long missionT0 = 0;       // START time of the mission (for the time stamps)
bool stepMode = false;          // T key: the mission waits for N before every step

// ======================================================
// STOP KEY: send X any time to stop the robot
// ======================================================
bool abortRequested() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      anyKeySeen = true;
    }
    if (c == 'X' || c == 'x') {
      missionAborted = true;
    }
  }

  // STOP button (pin 29 to GND): stops everything, same as X
  if (digitalRead(STOP_BUTTON_PIN) == LOW) {
    missionAborted = true;
  }

  if (missionAborted) {
    stopMotors();
  }

  return missionAborted;
}

// ======================================================
// STEP MODE (T key): the mission waits for N before every step.
// Use it to check each leg on the real arena with a tape.
// ======================================================
bool stepWait() {
  Serial.println(F("   send N = run this step    X = stop"));

  while (true) {
    if (digitalRead(STOP_BUTTON_PIN) == LOW) {
      missionAborted = true;
      stopMotors();
      return false;
    }

    if (Serial.available()) {
      char c = Serial.read();
      if (c >= 'a' && c <= 'z') c -= 32;
      if (c == 'N') return true;
      if (c == 'X') {
        missionAborted = true;
        stopMotors();
        return false;
      }
    }
  }
}

bool mgate(const __FlashStringHelper *name) {
  Serial.print(F("[t="));
  Serial.print((millis() - missionT0) / 1000);
  Serial.print(F(" s] "));
  Serial.println(name);

  if (!stepMode) return true;
  Serial.println(F("NEXT: (the step above)"));
  return stepWait();
}

// One mission move. After a straight move the gyro corrects the heading. After a move that ended on a
// WALL the wall has squared the robot, so the heading reference is taken from the wall instead.
bool mdriveCore(MoveType t, float mm, float slow, bool wall) {
  if (stepMode) {
    Serial.print(F("NEXT: "));
    Serial.print(moveName(t));
    Serial.print(F(" "));
    Serial.println((int)mm);
    if (!stepWait()) return false;
  }

  // ALIGN FIRST: before a straight move the gyro makes the robot straight, then it goes
  if (t != ROTATE_RIGHT && t != ROTATE_LEFT) {
    if (!fixHeading()) return false;
  }

  if (!drive(t, mm, slow)) return false;

  if (t == ROTATE_RIGHT || t == ROTATE_LEFT) return true;

#if USE_MPU
  if (wall && lastMoveHitWall && mpuOk) {
    settleWait(200);
    mpuUpdate();
    float off = yawDeg - headingRef;
    if (fabs(off) <= WALL_REF_MAX_DEG) {
      headingRef = yawDeg;
      Serial.println(F("   the wall has squared the robot: heading reference taken from it"));
      return true;
    }
    Serial.print(F("   wall reference NOT used: the robot is "));
    Serial.print(off);
    Serial.println(F(" deg from its heading (it was not flat on the wall). Turning it back by the gyro."));
  }
#endif

  return fixHeading();       // mandatory heading correction after every straight move
}

bool mdrive(MoveType t, float mm, float slow) {
  return mdriveCore(t, mm, slow, false);
}

// a move that pushes into a wall: when the robot touches the wall and stops, that is OK
bool mdriveWall(MoveType t, float mm, float slow) {
  moveAllowStall = true;
  bool ok = mdriveCore(t, mm, slow, true);
  moveAllowStall = false;
  return ok;
}

// ======================================================
// THE MISSION
// Samples are at random places: pickSample() finds each one.
// The drive numbers below are only the way to the place point.
// ======================================================
// slow-down distance for a move: long moves start slowing 40 mm before the end, short ones 10 mm
#define SZ(d) ((d) > 150 ? 40 : 10)

bool runMission() {
  stepCounter = 0;
  missionAborted = false;

#if !ARM_READY
  Serial.println();
#if !ARM_AT_BOOT
  Serial.println(F("!!! The mission will NOT start: ARM_AT_BOOT is 0 in Config.h (arm re-setup mode)."));
#else
  Serial.println(F("!!! The mission will NOT start: the arm numbers in Config.h are still the OLD ones (before the arm was opened)."));
  Serial.println(F("!!! If your arm already works with these numbers: in Config.h change  ARM_ALLOW_OLD_NUMBERS 0  to  ARM_ALLOW_OLD_NUMBERS 1  and upload."));
  Serial.println(F("!!! Otherwise teach the NEW numbers with ArmSetupTool (or key #) and write them into Config.h."));
#endif
  return false;
#endif

#if USE_MPU
  if (!mpuOk) {
    Serial.println(F("The gyro is not working: trying to start it again..."));
    mpuBegin();
  }
#if MPU_REQUIRED
  if (!mpuOk) {
    Serial.println();
    Serial.println(F("!!! The mission will NOT start: the GYRO (MPU6050) is not found. Without it the robot has NO heading correction"));
    Serial.println(F("!!! (no correction in the 90 degree turn and none while it drives). Check its wires: VCC, GND, SDA, SCL. Send @ to scan the I2C bus."));
    return false;
  }
#endif
#endif

  Serial.println();
  Serial.println(F("MISSION 1 START"));

  // gyro calibration: the robot must stand still
  missionT0 = millis();

  if (!alignReady()) {
    Serial.println(F("!!! WARNING: G setup is NOT done. The robot will NOT pick any sample!"));
  }

#if USE_MPU
  if (mpuOk) {
    Serial.println(F("Calibrating the gyro - take your hands away, do not touch the robot..."));
    if (!waitMs(1500)) return false;

    bool cal = false;
    for (int i = 0; i < 3 && !cal; i++) {
      cal = mpuCalibrate();
      if (!cal && !waitMs(700)) return false;
    }
    if (!cal) Serial.println(F("!!! GYRO NOT CALIBRATED (the robot moved). The earlier calibration is used."));

    headingRef = yawDeg;
  }
#endif

  setSuction(false);
  if (!armToInitial()) return false;

  // ================= PART A - QUARANTINE ZONE =================
  if (!mdrive(STRAFE_RIGHT, S01_STRAFE_RIGHT, SZ(S01_STRAFE_RIGHT))) return false;     // 1
  if (!mdrive(FORWARD,      S02_FORWARD,      SZ(S02_FORWARD)))      return false;     // 2
  if (!mdrive(P_TURN_RIGHT ? ROTATE_RIGHT : ROTATE_LEFT, TURN_90_WHEEL_MM, 90)) return false;   // 3  turn 90
  if (!mdriveWall(STRAFE_RIGHT, S04_STRAFE_RIGHT + S04_WALL_EXTRA_MM, 10)) return false;   // 4  into the wall corner
  if (!mdrive(BACKWARD,     S05_BACKWARD + S05_EXTRA_MM, 10))        return false;     // 5

  if (!mgate(F("STOP + CAMERA SCAN"))) return false;                                   // 6
  if (!armToScan()) return false;
  if (!waitMs(500)) return false;

  if (!mdrive(STRAFE_LEFT,  S07_STRAFE_LEFT,  SZ(S07_STRAFE_LEFT)))  return false;     // 7
  if (!mgate(F("PICK sample 1"))) return false;                                        // 8
  if (!pickSample(1)) return false;

  // ================= PART B - SAMPLE 1 TO LAB SLOT 1 =================
  if (!mdrive(STRAFE_LEFT,  S09_STRAFE_LEFT,  SZ(S09_STRAFE_LEFT)))  return false;     // 9
  if (P1_PLACE_FWD_MM > 0 && !mdrive(FORWARD, P1_PLACE_FWD_MM, 10)) return false;   // extra reach: forward
  if (!mgate(F("PLACE sample 1"))) return false;
  if (!placeSample(1)) return false;
  if (P1_PLACE_FWD_MM > 0 && !mdrive(BACKWARD, P1_PLACE_FWD_MM, 10)) return false;  // ... and back
  if (!mdriveWall(STRAFE_RIGHT, S13_STRAFE_RIGHT + S_RET_WALL_EXTRA_MM, 10)) return false;   // 13  back to the wall

  // ================= PART C - SAMPLE 2 TO LAB SLOT 2 =================
  if (!mgate(F("PICK sample 2"))) return false;                                        // 15
  if (!pickSample(2)) return false;
  if (!mdrive(STRAFE_LEFT,  S17_STRAFE_LEFT,  SZ(S17_STRAFE_LEFT)))  return false;     // 17
  if (P2_PLACE_FWD_MM > 0 && !mdrive(FORWARD, P2_PLACE_FWD_MM, 10)) return false;   // extra reach: forward
  if (!mgate(F("PLACE sample 2"))) return false;
  if (!placeSample(2)) return false;
  if (P2_PLACE_FWD_MM > 0 && !mdrive(BACKWARD, P2_PLACE_FWD_MM, 10)) return false;  // ... and back
  if (!mdriveWall(STRAFE_RIGHT, S21_STRAFE_RIGHT + S_RET_WALL_EXTRA_MM, 10)) return false;   // 21  back to the wall

  // ================= PART D - SAMPLE 3 TO LAB SLOT 3 =================
  if (!mgate(F("PICK sample 3"))) return false;                                        // 23
  if (!pickSample(3)) return false;
  if (!mdrive(STRAFE_LEFT,  S25_STRAFE_LEFT,  SZ(S25_STRAFE_LEFT)))  return false;     // 25
  if (P3_PLACE_FWD_MM > 0 && !mdrive(FORWARD, P3_PLACE_FWD_MM, 10)) return false;   // extra reach: forward
  if (!mgate(F("PLACE sample 3"))) return false;
  if (!placeSample(3)) return false;
  if (P3_PLACE_FWD_MM > 0 && !mdrive(BACKWARD, P3_PLACE_FWD_MM, 10)) return false;  // ... and back
  if (!mdriveWall(STRAFE_RIGHT, S29_STRAFE_RIGHT + S_RET_WALL_EXTRA_MM, 10)) return false;   // 29  back to the wall

  // ================= PART E - BEAM DEPLOYMENT AND FINISH =================
  if (!mdrive(BACKWARD,     S33_BACKWARD,     SZ(S33_BACKWARD)))     return false;     // 33
  if (!mgate(F("DEPLOY BEAMS (slab servo)"))) return false;                            // 34
  if (!openSlab()) return false;
  if (!mdrive(BACKWARD,     S35_BACKWARD,     SZ(S35_BACKWARD)))     return false;     // 35

  stopMotors();                                                                        // 36

  Serial.println();
  Serial.println(F("MISSION 1 COMPLETE - ALL STEPS FINISHED"));
  return true;
}

void startMission() {
  unsigned long t0 = millis();
  bool ok = runMission();

  stopMotors();

  if (!ok) {
    Serial.println();
    if (missionAborted) Serial.println(F("!!! STOPPED (X or STOP button) !!!"));
    else                Serial.println(F("!!! MISSION STOPPED (step failed) !!!"));
    Serial.println(F("Motors off. Arm holds. S = suction, I = arm to INITIAL, P = run again."));
  }

  Serial.print(F("Time: "));
  Serial.print((millis() - t0) / 1000);
  Serial.println(F(" s"));

  missionAborted = false;
}

// ======================================================
// SERIAL KEYS (only while waiting)
// ======================================================
void printHelp() {
  Serial.println(F("P or START button (pin 22) = START      X or STOP button (pin 29) = STOP"));
  Serial.println(F("G = GUIDED SETUP (do this first, once: ONE spot)"));
  Serial.println(F("K = find + align + pick test   L = place test   O = pick + place"));
  Serial.println(F("C = auto calibrate again (sample in view)   F = sweep find only"));
  Serial.println(F("H = scan pose   I = INITIAL   D = arm DOWN at the place pose"));
  Serial.println(F("+ / - = place pose lower arm   [ / ] = place pose upper arm"));
  Serial.println(F("8 2 4 6 = fine-tune where the nozzle lands (3 px), then test with K"));
  Serial.println(F("E = WHEEL TEST (robot on a STAND, wheels in the air): checks each motor + encoder"));
  Serial.println(F("Y = test the turn of mission step 3 (WHEELS MOVE, keep 40 cm clear)"));
  Serial.println(F(", / . = turn the arm base -10 / +10 (find which way is the lab side, see SWEEP_*_RANGE)"));
  Serial.println(F("$ = PICK MOTION TEST: the arm goes over the sample and down (no suction), where does the nozzle land?"));
  Serial.println(F("@ = I2C SCAN (is the gyro / arm driver / camera there?)   % = HOLD TEST (turn the robot by hand: it must turn back)"));
  Serial.println(F("# = ARM JOG: move each arm servo with keys and read its numbers (use after the arm was opened)"));
  Serial.println(F("0 = PREFLIGHT CHECK (before the match)   9 = CAMERA TEST   * = HEADING HOLD TEST   = : teach the LAB SIDE"));
  Serial.println(F("1 3 5 7 = try a LOWER scan pose (0/25/40/55 percent lower), shows the numbers for Config.h"));
  Serial.println(F("< > = scan pose SCAN_BOTTOM -5 / +5    ( ) = SCAN_TOP -5 / +5  (watch the arm, write the numbers into Config.h)"));
  Serial.println(F("M = gyro test (turn the robot by hand)   R = turn test 90 deg (WHEELS MOVE, works with or without the gyro)"));
  Serial.println(F("T = step mode ON/OFF: the mission waits for N before every step"));
  Serial.println(F("Q = measure speed reference (do once, half-charged battery, WHEELS MOVE ~30 cm)"));
  Serial.println(F("J = FORWARD 300 mm   V = STRAFE RIGHT 300 mm   U = BACKWARD 300 mm   A = DIAGONAL 300 mm   / = STRAFE LEFT 300 mm  (test drives, WHEELS MOVE)"));
  Serial.println(F("W = show settings   S = suction ON/OFF   B = slab servo test"));
  Serial.println(F("Z = forget saved settings"));
}

void showPlacePose() {
  Serial.print(F("PLACE_BOTTOM = "));
  Serial.print(cfg.placeBottom);
  Serial.print(F("   PLACE_TOP = "));
  Serial.print(cfg.placeTop);
  Serial.println(F("   (saved)"));
}

// + / -  : lower arm of the PLACE pose
void changePlaceHeight(int delta) {
  int wanted = cfg.placeBottom + delta;
  cfg.placeBottom = constrain(wanted, BOTTOM_MIN, BOTTOM_MAX);

  if (cfg.placeBottom != wanted) {
    Serial.print(F("LIMIT: lower arm allowed "));
    Serial.print(BOTTOM_MIN);
    Serial.print(F(" - "));
    Serial.println(BOTTOM_MAX);
  }

  settingsSave();
  armMoveTo(PLACE_BASE, cfg.placeBottom, cfg.placeTop, 300);
  showPlacePose();
}

// [ / ]  : upper arm of the PLACE pose
void changePlaceTop(int delta) {
  int wanted = cfg.placeTop + delta;
  cfg.placeTop = constrain(wanted, TOP_MIN, TOP_MAX);

  if (cfg.placeTop != wanted) {
    Serial.print(F("LIMIT: upper arm allowed "));
    Serial.print(TOP_MIN);
    Serial.print(F(" - "));
    Serial.println(TOP_MAX);
  }

  settingsSave();
  armMoveTo(PLACE_BASE, cfg.placeBottom, cfg.placeTop, 300);
  showPlacePose();
}

// 8 2 4 6 : move the taught target a few pixels, so the nozzle lands a bit
// differently. If it gets worse, press the opposite key. Test with K.
void nudgeTarget(int dx, int dy) {
  if (!cfg.targetSet) {
    Serial.println(F("No target yet. Do G first."));
    return;
  }

  cfg.targetX += dx;
  cfg.targetY += dy;
  settingsSave();

  Serial.print(F("TARGET now x="));
  Serial.print(cfg.targetX);
  Serial.print(F(" y="));
  Serial.println(cfg.targetY);
}

void forgetSettings() {
  settingsDefaults();
  settingsSave();
  Serial.println(F("Saved settings cleared."));
  showSettings();
}

// R: turn right 90 degrees by the gyro and show the result (WHEELS MOVE)
void turnTest() {
  Serial.println(F("TURN TEST: the robot turns right (the 90 degree turn of the mission). Keep 40 cm clear."));

  if (!mpuOk) {
    if (!drive(ROTATE_RIGHT, TURN_90_WHEEL_MM, 40)) return;
    Serial.println(F("Measure the angle it turned (use a square / protractor / the arena wall)."));
    Serial.println(F("If it is not 90:  new TURN_90_WHEEL_MM = old TURN_90_WHEEL_MM x 90 / measured angle"));
    return;
  }

  mpuUpdate();
  headingRef = yawDeg;
  float start = yawDeg;

  if (!drive(ROTATE_RIGHT, TURN_90_WHEEL_MM, 40)) return;

  delay(400);
  for (int i = 0; i < 20; i++) { mpuUpdate(); delay(5); }

  Serial.print(F("Turned "));
  Serial.print(yawDeg - start);
  Serial.println(F(" deg   (should be 90)"));
}

// ======================================================
// PREFLIGHT: run it before the match (key 0). It also runs at boot.
// It checks what the robot can check by itself and says GO / NOT READY.
// ======================================================
void preflight() {
  int problems = 0;

  Serial.println();
  Serial.print(F("===== PREFLIGHT CHECK  ("));
  Serial.print(F(BUILD_NAME));
  Serial.println(F(") ====="));

  // arm driver (pump switch, slab servo and the arm are on it)
  Wire.beginTransmission(PCA9685_ADDR);
  if (Wire.endTransmission() == 0) Serial.println(F("OK   arm driver PCA9685"));
  else { Serial.println(F("FAIL arm driver PCA9685 not found (arm, pump and slab will not work)")); problems++; }

  // camera
  visionOk = gv2Present();
  if (visionOk) Serial.println(F("OK   camera Grove Vision AI V2"));
  else { Serial.println(F("FAIL camera not found (no sample can be seen)")); problems++; }

  // gyro
#if USE_MPU
  if (mpuOk) {
    Serial.print(F("OK   gyro MPU6050   (since power-up: "));
    Serial.print(mpuFailTotal);
    Serial.print(F(" failed reads, "));
    Serial.print(mpuGlitchTotal);
    Serial.println(F(" wild readings ignored)"));
  }
  else { Serial.println(F("WARN gyro not found: the robot drives by the encoders only (less straight)")); problems++; }
#endif

  // arm numbers
#if ARM_READY
  Serial.println(F("OK   arm numbers in Config.h are the new ones"));
#else
  Serial.println(F("FAIL arm numbers in Config.h are still the OLD ones (or ARM_AT_BOOT is 0): teach them again, see ArmSetupTool"));
  problems++;
#endif

  // G setup
  if (alignReady()) Serial.println(F("OK   G setup done (the robot can pick)"));
  else { Serial.println(F("FAIL G setup NOT done: send G, otherwise NO sample is picked")); problems++; }

  // slab servo
#if USE_SLAB_SERVO
  Serial.println(F("OK   slab servo is enabled"));
#else
  Serial.println(F("WARN slab servo is NOT enabled (USE_SLAB_SERVO 0): step 34 only waits"));
  problems++;
#endif

  // buttons
  if (digitalRead(STOP_BUTTON_PIN) == LOW) { Serial.println(F("FAIL the STOP button is pressed / the wire is shorted")); problems++; }
  else Serial.println(F("OK   STOP button released"));
  if (digitalRead(START_BUTTON_PIN) == LOW) { Serial.println(F("FAIL the START button is pressed / the wire is shorted")); problems++; }
  else Serial.println(F("OK   START button released"));

  // settings that should be looked at once
  Serial.print(F("INFO step 3 turns "));
  Serial.print(P_TURN_RIGHT ? F("RIGHT") : F("LEFT"));
  Serial.print(F(" (ROTATION_MIRRORED="));
  Serial.print(ROTATION_MIRRORED);
  Serial.println(F(")   -> check with Y that the robot really turns LEFT"));
  Serial.print(F("INFO arm sweep  + side "));
  Serial.print(SWEEP_PLUS_RANGE);
  Serial.print(F("   - side "));
  Serial.print(SWEEP_MINUS_RANGE);
  Serial.println(F("   (the lab side should be small, like 20)"));
  Serial.print(F("INFO arm sweep lab side: "));
  if (labDir > 0) Serial.println(F("+ side (taught)  -> only a little sweep that way"));
  else if (labDir < 0) Serial.println(F("- side (taught)  -> only a little sweep that way"));
  else { Serial.println(F("NOT taught: send =  and turn the arm to the lab (front-left)")); }
  Serial.print(F("INFO place pose  bottom "));
  Serial.print(cfg.placeBottom);
  Serial.print(F("  top "));
  Serial.println(cfg.placeTop);

  if (problems == 0) Serial.println(F("===== PREFLIGHT: ALL GOOD - READY ====="));
  else {
    Serial.print(F("===== PREFLIGHT: "));
    Serial.print(problems);
    Serial.println(F(" POINT(S) TO FIX (see above) ====="));
  }
}

// The scan pose that is being tuned with the keys  < > ( )  (starts at the compiled SCAN pose,
// and follows the keys 1 3 5 7)
int tuneBottom = SCAN_BOTTOM;
int tuneTop = (SCAN_TOP > TOP_MIN) ? SCAN_TOP : TOP_MIN;

// < > : SCAN_BOTTOM -5 / +5      ( ) : SCAN_TOP -5 / +5     (the camera checks the sample every time)
void scanTune(int dBottom, int dTop) {
  tuneBottom = constrain(tuneBottom + dBottom, BOTTOM_MIN, BOTTOM_MAX);
  tuneTop    = constrain(tuneTop + dTop, TOP_MIN, TOP_MAX);

  armMoveTo(SCAN_BASE, tuneBottom, tuneTop, 300);

  Serial.print(F("SCAN POSE:  SCAN_BOTTOM "));
  Serial.print(tuneBottom);
  Serial.print(F("   SCAN_TOP "));
  Serial.println(tuneTop);

  if (!waitMs(400)) return;

  int x, y;
  if (lookForSample(x, y, 800)) Serial.println(F("   the camera SEES the sample"));
  else                          Serial.println(F("   the camera does NOT see a sample"));

  Serial.print(F(">>> WRITE INTO Config.h:   #define SCAN_BOTTOM "));
  Serial.print(tuneBottom);
  Serial.print(F("    #define SCAN_TOP "));
  Serial.println(tuneTop);
}

// 1 3 5 7 : try a LOWER scan pose. The arm goes down this share of the way towards the pick pose
// (0 / 25 / 40 / 55 percent), the camera looks, and the numbers to write into Config.h are shown.
void scanTrial(float frac) {
  int tb = SCAN_BOTTOM;
  int tt = (SCAN_TOP > TOP_MIN) ? SCAN_TOP : TOP_MIN;      // SCAN_TOP below TOP_MIN works as TOP_MIN

  int b = (int)(tb + frac * (cfg.pickBottom - tb) + 0.5);
  int t = (int)(tt + frac * (cfg.pickTop - tt) + 0.5);

  tuneBottom = b;
  tuneTop = t;
  armMoveTo(SCAN_BASE, b, t, 800);

  Serial.print(F("SCAN POSE TRY ("));
  Serial.print((int)(frac * 100));
  Serial.print(F(" percent lower):  SCAN_BOTTOM "));
  Serial.print(b);
  Serial.print(F("   SCAN_TOP "));
  Serial.println(t);

  if (!waitMs(500)) return;

  int x, y;
  if (lookForSample(x, y, 1500)) {
    Serial.println(F("   the camera SEES the sample from this height"));
  } else {
    Serial.println(F("   the camera does NOT see a sample from this height (is a sample in front of the arm?)"));
  }

  Serial.print(F(">>> WRITE INTO Config.h:   #define SCAN_BOTTOM "));
  Serial.print(tuneBottom);
  Serial.print(F("    #define SCAN_TOP "));
  Serial.println(tuneTop);
}

// * : HEADING HOLD TEST. The robot strafes right 300 mm twice, once with the gyro help and once
// without, and shows how crooked it got. It tells you if the help works or has the wrong sign.
void assistTest() {
  if (!mpuOk) {
    Serial.println(F("The gyro (MPU6050) is not working, so there is no heading help."));
    return;
  }

  Serial.println(F("HEADING HOLD TEST: the robot strafes right 300 mm and back, twice. Keep 40 cm free on both sides."));

  float errOn = 0, errOff = 0;

  for (int pass = 0; pass < 2; pass++) {
    assistEnabled = (pass == 0);

    mpuUpdate();
    headingRef = yawDeg;

    assistLog = (pass == 0);
    bool okDrive = drive(STRAFE_RIGHT, 300, 40);
    assistLog = false;
    if (!okDrive) { assistEnabled = true; return; }
    settleWait(200);
    mpuUpdate();
    float e = yawDeg - headingRef;
    if (pass == 0) errOn = e; else errOff = e;

    Serial.print(assistEnabled ? F("  WITH the gyro help:    heading error ") : F("  WITHOUT the gyro help: heading error "));
    Serial.print(e);
    Serial.println(F(" deg"));

    assistEnabled = true;
    if (!fixHeading()) return;
    if (!drive(STRAFE_LEFT, 300, 40)) return;
    if (!fixHeading()) return;
  }

  assistEnabled = true;

  float a1 = fabs(errOn), a0 = fabs(errOff);
  if (a1 < 1.0 && a0 < 1.0) {
    Serial.println(F("RESULT: the robot barely turns while strafing. All fine."));
  } else if (a1 <= a0 * 0.7) {
    Serial.println(F("RESULT: the gyro help WORKS (the robot is straighter with it)."));
  } else if (a1 >= a0 * 1.3) {
    Serial.println(F("RESULT: the help makes it WORSE. In Config.h set  ASSIST_SIGN  to -1  (or +1 if it is -1 now), upload, and run * again."));
  } else {
    Serial.println(F("RESULT: no clear difference. Make ASSIST_KP_PWM bigger (for example 20)."));
  }
}

// $ : PICK MOTION TEST (no suction, the robot does not drive). The arm goes from the scan pose over the
// sample and down onto the taught pick pose, waits so you can look, and goes back up.
// Put a sample under the nozzle spot (the place you used in G) and watch where the nozzle lands.
void pickMotionTest() {
  Serial.println(F("PICK MOTION TEST: the arm goes over the sample and down. Keep your hands away."));

  if (!armToScan()) return;
  if (!waitMs(300)) return;

  float base = armBase;

  Serial.print(F("pick pose:  bottom "));
  Serial.print(cfg.pickBottom);
  Serial.print(F("   top "));
  Serial.println(cfg.pickTop);

  if (!armOver(base, cfg.pickBottom, cfg.pickTop)) return;
  Serial.println(F("arm is OVER the sample (hover)"));
  if (!waitMs(800)) return;

  if (!armDown(base, cfg.pickBottom, cfg.pickTop)) return;
  Serial.println(F("arm is DOWN. Where is the nozzle? On the sample, or behind / in front of it?  (3 s)"));
  if (!waitMs(3000)) return;

  armOver(base, cfg.pickBottom, cfg.pickTop);
  armToScan();
}

// @ : I2C SCAN. Shows every chip that answers on the bus. You must see 0x40 (arm driver), 0x62 (camera)
// and 0x68 or 0x69 (gyro MPU6050).
void i2cScan() {
  Serial.println(F("I2C SCAN:"));
  int n = 0;
  bool gyro = false, arm = false, cam = false;

  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;
    n++;
    Serial.print(F("   found 0x"));
    Serial.print(a, HEX);
    if (a == PCA9685_ADDR) { Serial.print(F("  = arm driver PCA9685")); arm = true; }
    else if (a == 0x62)    { Serial.print(F("  = camera Grove Vision AI V2")); cam = true; }
    else if (a == 0x68 || a == 0x69) { Serial.print(F("  = gyro MPU6050")); gyro = true; }
    else if (a == 0x70)    { Serial.print(F("  = PCA9685 all-call address (normal)")); }
    Serial.println();
  }

  if (n == 0) Serial.println(F("   NOTHING answers: the I2C bus is dead (power, SDA / SCL wires)."));
  if (!arm)  Serial.println(F("   !!! the arm driver (0x40) is missing"));
  if (!cam)  Serial.println(F("   !!! the camera (0x62) is missing"));
  if (!gyro) Serial.println(F("   !!! the GYRO (0x68 / 0x69) is MISSING: no heading correction at all. Check the MPU6050 wires (VCC, GND, SDA, SCL)."));
}

// % : HOLD TEST - the visible proof that the gyro works. The robot stands on the floor. Turn it BY HAND
// 10 to 20 degrees (left or right) and let go: it must turn itself back. Runs 20 seconds. X = stop.
void holdTest() {
  if (!mpuOk) {
    Serial.println(F("The gyro (MPU6050) is NOT working, so nothing can be corrected. Send @ to scan the I2C bus."));
    return;
  }

  Serial.println(F("HOLD TEST (20 s): turn the robot by hand 10 to 20 degrees and let go. It must turn itself back. X = stop."));

  settleWait(300);
  mpuUpdate();
  headingRef = yawDeg;

  unsigned long t0 = millis();
  unsigned long tp = 0;

  while (millis() - t0 < 20000) {
    if (abortRequested()) { stopMotors(); return; }
    mpuUpdate();

    float err = yawDeg - headingRef;

    if (millis() - tp >= 500) {
      tp = millis();
      Serial.print(F("   heading error "));
      Serial.print(err);
      Serial.println(F(" deg"));
    }

    if (fabs(err) > 2.0) {
      Serial.println(F("   you turned it: the robot corrects it..."));
      if (!fixHeading()) return;
    }
  }

  Serial.println(F("HOLD TEST finished."));
}

void checkSerial() {
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c >= 'a' && c <= 'z') c -= 32;

  switch (c) {
    case 'P': startMission();          break;
    case 'G': guidedSetup();           break;
    case 'C': calibrateCamera();       break;
    case 'F': findTest();              break;
    case 'K': pickSample(0);           break;
    case 'L': placeSample(0);          break;
    case 'O': if (pickSample(0)) placeSample(0); break;
    case 'H': armToScan();             break;
    case 'I': armToInitial();          break;
    case 'D': armMoveTo(PLACE_BASE, cfg.placeBottom, cfg.placeTop, ARM_DOWN_MS); showPlacePose(); break;
    case '-': changePlaceHeight(-5);   break;   // smaller value = lower
    case '+': changePlaceHeight(+5);   break;
    case '[': changePlaceTop(-5);      break;
    case ']': changePlaceTop(+5);      break;
    case '8': nudgeTarget(0, -3);        break;
    case '2': nudgeTarget(0, +3);        break;
    case '4': nudgeTarget(-3, 0);        break;
    case '6': nudgeTarget(+3, 0);        break;
    case 'Q': speedTest();             break;
    case 'E': wheelTest();             break;
    case 'M': mpuTest();               break;
    case '0': preflight();             break;
    case '*': assistTest();            break;
    case '%': holdTest();              break;
    case '@': i2cScan();               break;
    case '=': labSideTeach();          break;
    case '#': armJog();                break;
    case '$': pickMotionTest();        break;
    case '9': cameraTest();            break;
    case '1': scanTrial(0.00);         break;
    case '3': scanTrial(0.25);         break;
    case '5': scanTrial(0.40);         break;
    case '7': scanTrial(0.55);         break;
    case '<': scanTune(-5, 0);         break;
    case '>': scanTune(+5, 0);         break;
    case '(': scanTune(0, -5);         break;
    case ')': scanTune(0, +5);         break;
    case ',': armMoveTo((int)armBase - 10, SCAN_BOTTOM, SCAN_TOP, 300);
              Serial.print(F("arm base = ")); Serial.println((int)armBase); break;
    case '.': armMoveTo((int)armBase + 10, SCAN_BOTTOM, SCAN_TOP, 300);
              Serial.print(F("arm base = ")); Serial.println((int)armBase); break;
    case 'R': turnTest();              break;
    case 'Y':                          // the exact turn of mission step 3
      Serial.println(P_TURN_RIGHT ? F("STEP 3 TURN TEST: turning RIGHT 90") : F("STEP 3 TURN TEST: turning LEFT 90"));
      drive(P_TURN_RIGHT ? ROTATE_RIGHT : ROTATE_LEFT, TURN_90_WHEEL_MM, 40);
      break;
    case 'T': stepMode = !stepMode; Serial.println(stepMode ? F("STEP MODE ON: the mission waits for N before each step") : F("STEP MODE OFF")); break;
    case 'J': drive(FORWARD, 300, 40);      break;   // test: measure with a tape
    case 'V': drive(STRAFE_RIGHT, 300, 40); break;   // test: measure with a tape
    case 'U': drive(BACKWARD, 300, 40);     break;   // test: measure with a tape
    case '/': drive(STRAFE_LEFT, 300, 40);  break;   // test: strafe left along the reference line
    case 'A': drive(DIAGONAL_FRONT_RIGHT, 300, 40); break;   // test: measure with a tape
    case 'W': showSettings();          break;
    case 'S': setSuction(!suctionActive); break;
    case 'B': slabTest();              break;
    case 'Z': forgetSettings();        break;
    case '?': printHelp();             break;
  }

  missionAborted = false;   // an X sent while idle must not block the next run
}

void checkStartButton() {
  bool state = digitalRead(START_BUTTON_PIN);
  if (lastButtonState == HIGH && state == LOW) {
    delay(30);                                  // debounce
    if (digitalRead(START_BUTTON_PIN) == LOW) startMission();
  }
  lastButtonState = state;
}

// ======================================================
// SETUP + LOOP
// ======================================================
void setup() {
  Serial.begin(115200);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);

  pinMode(M1_PWM, OUTPUT); pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT); pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);
  pinMode(M4_PWM, OUTPUT); pinMode(M4_IN1, OUTPUT); pinMode(M4_IN2, OUTPUT);

  stopMotors();
  encodersBegin();

  settingsLoad();
  speedLoad();
  labSideLoad();

  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("MISSION 1 - BOTICS ALPHA ST68"));
  Serial.println(F("================================"));
  printHelp();
  Serial.println();

  // ---- I2C health check (arm driver and camera share this bus) ----
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  delay(10);
  bool sdaHigh = digitalRead(I2C_SDA_PIN);
  bool sclHigh = digitalRead(I2C_SCL_PIN);

  Serial.print(F("I2C lines: SDA="));
  Serial.print(sdaHigh ? F("HIGH") : F("LOW"));
  Serial.print(F("  SCL="));
  Serial.println(sclHigh ? F("HIGH") : F("LOW"));

  if (!sdaHigh || !sclHigh) {
    Serial.println(F("!!! I2C BUS IS STUCK LOW. Arm and camera cannot work."));
    Serial.println(F("!!! Check: Grove powered? Level shifter LV = 3.3V? Loose SDA/SCL wire?"));
  }

  Wire.begin();
  Wire.setWireTimeout(25000, true);   // never hang if I2C gets stuck

  Wire.beginTransmission(PCA9685_ADDR);
  armDriverOk = (Wire.endTransmission() == 0);

  Serial.print(F("Arm driver PCA9685 (0x40)... "));
  Serial.println(armDriverOk ? F("OK") : F("NOT FOUND"));

  if (!armDriverOk) {
    Serial.println(F("!!! THE ARM WILL NOT MOVE until the PCA9685 answers on I2C."));
  }

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(100);

  i2cScan();

#if USE_MPU
  Serial.print(F("MPU6050 gyro... "));
  if (mpuBegin()) {
    Serial.print(F("OK at 0x"));
    Serial.print(mpuAddress, HEX);
    Serial.println(F("  - keep the robot still"));
    mpuCalibrate();
  } else {
    Serial.println(F("!!! NOT FOUND !!! The robot has NO heading correction (no gyro help, no exact 90 degree turn). Check the MPU6050 wires."));
  }
#endif

#if SUCTION_USE_MEGA_PIN
  digitalWrite(SUCTION_PIN, SUCTION_INVERT ? HIGH : LOW);   // pump OFF first
  pinMode(SUCTION_PIN, OUTPUT);
#endif

  setSuction(false);   // suction OFF at power-up

#if USE_SLAB_SERVO
  pwm.setPWM(SLAB_SERVO, 0, SLAB_CLOSED);
#endif

#if ARM_READY
  // Arm is resting at IDLE: hold it there, then go to INITIAL
  armBase = IDLE_BASE;
  armBottom = IDLE_BOTTOM;
  armTop = IDLE_TOP;
  armApply();
#else
  // ARM RE-SETUP: nothing is sent to the arm servos at power-up (the old numbers are wrong)
  armBase = INITIAL_BASE;
  armBottom = INITIAL_BOTTOM;
  armTop = INITIAL_TOP;
#endif

  // Grove needs a few seconds to boot, so retry
  Serial.print(F("Connecting Grove Vision AI V2... "));
  for (int i = 0; i < 10 && !visionOk; i++) {
    visionOk = gv2Present();
    if (!visionOk) delay(300);
  }
  Serial.println(visionOk ? F("OK") : F("NOT FOUND"));

  showSettings();

  Serial.print(F("BUILD CHECK  factors: forward="));
  Serial.print(FORWARD_FACTOR);
  Serial.print(F("  back="));
  Serial.print(BACKWARD_FACTOR);
  Serial.print(F("  strafe="));
  Serial.print(STRAFE_FACTOR);
  Serial.print(F("  diagonal="));
  Serial.print(DIAGONAL_FACTOR);
  Serial.print(F("   SAFE_MODE="));
  Serial.println(SAFE_MODE);
  Serial.print(F("BUILD: "));
  Serial.println(F(BUILD_NAME));
  Serial.print(F("BUILD CHECK  gyro help="));
  Serial.print(USE_MPU ? F("ON") : F("OFF"));
  Serial.print(F("  sweep +"));
  Serial.print(SWEEP_PLUS_RANGE);
  Serial.print(F(" / -"));
  Serial.print(SWEEP_MINUS_RANGE);
  Serial.print(F("  wheel search rows="));
  Serial.print(SEARCH_ROWS);
  Serial.println(F(" step(s) forward"));
  Serial.print(F("BUILD CHECK  step 3 turn = "));
  Serial.println(P_TURN_RIGHT ? F("RIGHT") : F("LEFT"));

  Serial.print(F("Speed reference: "));
  Serial.print(refCps);
  Serial.println(F(" counts/s  (send Q to measure it once)"));

#if ARM_READY
  armMoveTo(INITIAL_BASE, INITIAL_BOTTOM, INITIAL_TOP, 3000);
#else
  Serial.println();
#if !ARM_AT_BOOT
  Serial.println(F("!!! ARM_AT_BOOT is 0: the arm was NOT moved at power-up."));
#else
  Serial.println(F("!!! The arm numbers in Config.h are still the OLD ones: the arm was NOT moved at power-up and the mission will not start."));
  Serial.println(F("!!! Teach the NEW numbers (ArmSetupTool, or key #), write them into Config.h, upload again."));
#endif
#endif
  missionAborted = false;

#if AUTO_START
  Serial.println();
  Serial.print(F("AUTO START in "));
  Serial.print(AUTO_START_DELAY_MS / 1000);
  Serial.println(F(" s. Send any key NOW to cancel and stay in TEST mode."));

  unsigned long t0 = millis();
  while (!anyKeySeen && millis() - t0 < AUTO_START_DELAY_MS) {
    abortRequested();           // also notices any key
    delay(10);
  }

  missionAborted = false;

  if (!anyKeySeen) {
    startMission();
  } else {
    Serial.println(F("AUTO START cancelled. TEST mode: wheels stay still."));
  }
#endif

  preflight();

  Serial.println(F("READY. Press the button or send P to start Mission 1."));
}

void loop() {
  checkSerial();
  checkStartButton();
}
