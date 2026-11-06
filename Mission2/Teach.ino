// ======================================================
// Teach.ino  -  TEACH SLOTS, ZONES, ARM POSES (saved in EEPROM)
//
// JOG KEYS (type the letters and press Enter; "www" = 3 steps):
//   wheels:  w = forward   s = back   a = left   d = right     m = step 10 / 5 mm
//   arm:     j / l = base - / +
//            i / k = bottom + / -
//            u / o = top + / -                                 z = step 5 / 2 / 1
//   v = claw CLOSE/OPEN (test the grip)   c = camera look
//   y = SAVE          q = quit (not saved)
// ======================================================
#include "Config.h"

static float jogF = 0, jogS = 0;       // wheel jog (mm forward / mm left) since the teach started

static void jogStatus() {
  Serial.print(F("   ARM base="));
  Serial.print((int)armBase);
  Serial.print(F(" bottom="));
  Serial.print((int)armBottom);
  Serial.print(F(" top="));
  Serial.print((int)armTop);
  Serial.print(F("   | robot moved fwd="));
  Serial.print(jogF, 0);
  Serial.print(F(" left="));
  Serial.print(jogS, 0);
  Serial.println(F(" mm"));
}

static void jogHelp(bool wheels) {
  if (wheels) Serial.println(F("   WHEELS: w fwd  s back  a left  d right   m = step 10/5 mm"));
  Serial.println(F("   ARM:    j/l base -/+   i/k bottom +/-   u/o top +/-   z = step 5/2/1"));
  Serial.println(F("   v = claw CLOSE/OPEN (grip test)   c = camera look   y = SAVE   q = quit"));
}

// wheel jog: lift the arm a little, move, put it back
static bool jogWheel(MoveType t, float mm) {
  int b = (int)armBase, bo = (int)armBottom, tp = (int)armTop;
  if (!armOver(b, bo, tp)) return false;
  if (!smallMove(t, mm)) return false;
  if (!fixHeading()) return false;
  if (t == FORWARD)      { jogF += mm; navX += mm; }
  if (t == BACKWARD)     { jogF -= mm; navX -= mm; }
  if (t == STRAFE_LEFT)  { jogS += mm; navY += mm; }
  if (t == STRAFE_RIGHT) { jogS -= mm; navY -= mm; }
  return armMoveTo(b, bo, tp, 600);
}

static void cameraLookHere() {
  int b = (int)armBase, bo = (int)armBottom, tp = (int)armTop;
  if (!armOver(b, bo, tp) || !armLiftHere()) return;
  waitMs(SETTLE_MS);
  int x, y;
  setAim(IMAGE_CENTER_X, IMAGE_CENTER_Y);
  if (lookForSample(x, y, LOOK_MS)) {
    Serial.print(F("   camera: "));
    Serial.print(colorName(lastColor));
    Serial.print(F(" at x="));
    Serial.print(x);
    Serial.print(F(" y="));
    Serial.println(y);
  } else {
    Serial.println(F("   camera: no patient seen"));
  }
  armOver(b, bo, tp);
  armMoveTo(b, bo, tp, 600);
}

// returns 1 = saved, 0 = quit, -1 = stopped
static int jogLoop(bool wheels) {
  int armStep = 5;
  float wStep = 10;
  char line[40];
  jogHelp(wheels);
  jogStatus();

  while (true) {
    if (!readLine(line, sizeof(line), true)) return -1;
    for (byte i = 0; line[i]; i++) {
      char c = line[i];
      if (c >= 'A' && c <= 'Z') c += 32;
      float nb = armBase, nbo = armBottom, nt = armTop;
      bool armMove = false;

      switch (c) {
        case 'y': return 1;
        case 'q': case 'x': Serial.println(F("   quit - NOT saved")); return 0;
        case 'h': case '?': jogHelp(wheels); break;
        case 'z': armStep = (armStep == 5) ? 2 : (armStep == 2) ? 1 : 5;
                  Serial.print(F("   arm step ")); Serial.println(armStep); break;
        case 'm': wStep = (wStep == 10) ? 5 : 10;
                  Serial.print(F("   wheel step ")); Serial.print(wStep, 0); Serial.println(F(" mm")); break;
        case 'j': nb -= armStep; armMove = true; break;
        case 'l': nb += armStep; armMove = true; break;
        case 'i': nbo += armStep; armMove = true; break;
        case 'k': nbo -= armStep; armMove = true; break;
        case 'u': nt += armStep; armMove = true; break;
        case 'o': nt -= armStep; armMove = true; break;
        case 'v': setSuction(!suctionActive); break;
        case 'c': cameraLookHere(); break;
        case 'w': if (wheels && !jogWheel(FORWARD, wStep)) return -1; break;
        case 's': if (wheels && !jogWheel(BACKWARD, wStep)) return -1; break;
        case 'a': if (wheels && !jogWheel(STRAFE_LEFT, wStep)) return -1; break;
        case 'd': if (wheels && !jogWheel(STRAFE_RIGHT, wStep)) return -1; break;
        default: break;
      }
      if (armMove && !armMoveTo((int)nb, (int)nbo, (int)nt, 200)) return -1;
      missionAborted = false;
    }
    jogStatus();
  }
}

static bool teachStart() {
  missionReset();
  setSuction(false);
  Serial.println(F("Robot must stand on H CENTER, facing the hospital. Gyro calibrating..."));
  return gyroStart(0);
}

// Wheel calibration: move CAL_STRAFE_MM with "there", look, move back with "back".
// vx, vy = how many pixels the patient moves in the picture per mm.
// 1 = OK, 0 = failed (not seen / no movement), -1 = stopped
static int calWheel(MoveType there, MoveType back, int x0, int y0, float &vx, float &vy) {
  int xa, ya, wa, ha;
  waitMs(SETTLE_MS);
  setAim(x0, y0);
  if (!lookPrecise(xa, ya, wa, ha, SCAN_TIMEOUT_MS)) return missionAborted ? -1 : 0;
  if (!smallMove(there, CAL_STRAFE_MM) || !fixHeading()) return -1;
  waitMs(SETTLE_MS);
  int x2, y2, w2, h2;
  setAim(xa, ya);
  bool seen = lookPrecise(x2, y2, w2, h2, SCAN_TIMEOUT_MS);
  if (!smallMove(back, CAL_STRAFE_MM) || !fixHeading()) return -1;
  if (!seen) return 0;
  vx = (float)(x2 - xa) / CAL_STRAFE_MM;
  vy = (float)(y2 - ya) / CAL_STRAFE_MM;
  if (fabs(vx) + fabs(vy) <= 0.2) return 0;
  Serial.print(F("   "));
  Serial.print(moveName(there));
  Serial.print(F(" cal: "));
  Serial.print(sqrt(vx * vx + vy * vy), 2);
  Serial.print(F(" px per mm  (dx "));
  Serial.print(vx, 2);
  Serial.print(F("  dy "));
  Serial.print(vy, 2);
  Serial.println(F(")"));
  return 1;
}

// ---------------- TEACH ONE PATIENT SLOT ----------------
bool teachSlot(byte s) {
  Serial.println();
  Serial.print(F("=== TEACH SLOT "));
  Serial.print(s + 1);
  Serial.println(slotIsRight(s) ? F(" (RIGHT) ===") : F(" (LEFT) ==="));
  Serial.println(F("Put a patient on this dot first."));
  if (!teachStart()) return false;

  if (!armToInitial()) return false;
  if (!navToSlot(s)) return false;

  Pose3 p = st.slot[s].ok ? st.slot[s].pick : slotDefaultPose(s);
  if (!armScanAt(p.b)) return false;
  waitMs(SETTLE_MS);
  int x, y;
  setAim(IMAGE_CENTER_X, IMAGE_CENTER_Y);
  if (lookForSample(x, y, LOOK_MS)) {
    Serial.print(F("Camera sees "));
    Serial.print(colorName(lastColor));
    Serial.print(F(" at x="));
    Serial.print(x);
    Serial.print(F(" y="));
    Serial.println(y);
  } else {
    Serial.println(F("Camera does not see the patient yet - move closer with the wheels / arm."));
  }

  if (!armOver(p.b, p.bo, p.t)) return false;
  if (!armDown(p.b, p.bo, p.t)) return false;

  jogF = 0; jogS = 0;
  Serial.println(F(">> Claw is OPEN. Move the robot (wheels) and the arm until the open claw"));
  Serial.println(F(">> is AROUND the patient (fingers low enough). Test with v (close / open). Then y."));
  int r = jogLoop(true);
  if (r < 0) return false;
  setSuction(false);

  if (r == 1) {
    SlotData &sd = st.slot[s];
    sd.dx = (sd.ok ? sd.dx : 0) + jogF;
    sd.dy = (sd.ok ? sd.dy : 0) + jogS;
    sd.pick = {(int)armBase, (int)armBottom, (int)armTop};
    sd.ok = 1;

    // remember where the camera sees it (for the fine correction + the pick check)
    Serial.println(F("Recording the camera picture - do NOT touch the patient..."));
    if (!armOver(sd.pick.b, sd.pick.bo, sd.pick.t)) return false;
    if (!armLiftHere()) return false;
    waitMs(600);
    setAim(IMAGE_CENTER_X, IMAGE_CENTER_Y);
    int x0, y0, w0, h0;
    if (lookPrecise(x0, y0, w0, h0, SCAN_TIMEOUT_MS)) {
      sd.px = x0; sd.py = y0; sd.hasPix = 1;
      // camera calibration 1: turn the base a little
      if (armMoveTo(sd.pick.b + CAL_BASE_UNITS, st.scan.bo, st.scan.t, 400) && waitMs(500)) {
        int x1, y1, w1, h1;
        setAim(x0, y0);
        if (lookPrecise(x1, y1, w1, h1, SCAN_TIMEOUT_MS)) {
          float bx = (float)(x1 - x0) / CAL_BASE_UNITS, by = (float)(y1 - y0) / CAL_BASE_UNITS;
          if (fabs(bx) + fabs(by) > 0.2) { st.bx = bx; st.by = by; st.baseCal = 1; }
        }
      }
      if (!armScanAt(sd.pick.b)) return false;

      // camera calibration 2 + 3: move the robot a little LEFT and back, then FORWARD and back
      st.strafeCal[s] = 0;
      st.fwdCal[s] = 0;
#if FINE_WHEELS
      Serial.println(F("Wheel calibration: robot moves a little LEFT and back, then FORWARD and back..."));
      float vx, vy;
      int rc = calWheel(STRAFE_LEFT, STRAFE_RIGHT, x0, y0, vx, vy);
      if (rc < 0) return false;
      if (rc == 1) { st.sx[s] = vx; st.sy[s] = vy; st.strafeCal[s] = 1; }
      rc = calWheel(FORWARD, BACKWARD, x0, y0, vx, vy);
      if (rc < 0) return false;
      if (rc == 1) { st.fwx[s] = vx; st.fwy[s] = vy; st.fwdCal[s] = 1; }
      if (!st.strafeCal[s]) Serial.println(F("   LEFT/RIGHT cal FAILED"));
      if (!st.fwdCal[s])    Serial.println(F("   FORWARD/BACK cal FAILED"));
#endif
    } else {
      sd.hasPix = 0;
      Serial.println(F("(camera did not see it from the scan height - pick works, but no fine correction)"));
    }
    storeSave();
    Serial.print(F("SLOT "));
    Serial.print(s + 1);
    Serial.println(F(" SAVED."));
  }

  if (!armToInitial()) return false;
  return navReturnNode();
}

// ---------------- TEACH ONE DROP ZONE ----------------
bool teachZone(Zone z) {
  Serial.println();
  Serial.print(F("=== TEACH ZONE "));
  Serial.print(zoneName(z));
  Serial.println(F(" ==="));
  if (!teachStart()) return false;
  if (!armToInitial()) return false;
  if (!navToZone(z, 0)) return false;

  Pose3 p = st.zone[z].place;
  // Recovery is BEHIND the robot: start with the RIGHT arm (reaching down), not the front one
  if (z == ZONE_RECOVERY && !st.zone[z].ok) p = {st.scan.b, PICK_BOTTOM, PICK_TOP};
  if (!armOver(p.b, p.bo, p.t)) return false;
  if (!armDown(p.b, p.bo, p.t)) return false;

  jogF = 0; jogS = 0;
  Serial.println(F(">> Move robot + arm so the patient will be DROPPED right inside the zone. Then y."));
  int r = jogLoop(true);
  if (r < 0) return false;
  if (r == 1) {
    ZoneData &zd = st.zone[z];
    zd.dx = (zd.ok ? zd.dx : 0) + jogF;
    zd.dy = (zd.ok ? zd.dy : 0) + jogS;
    zd.place = {(int)armBase, (int)armBottom, (int)armTop};
    zd.ok = 1;
    storeSave();
    Serial.print(zoneName(z));
    Serial.println(F(" SAVED."));
  }
  if (!armToInitial()) return false;
  return navReturnNode();
}

// ---------------- TEACH AN ARM POSE (no driving) ----------------
// which: 0 idle, 1 initial, 2 scan right, 3 scan left (base only)
bool teachPose(byte which) {
  Pose3 p;
  if (which == 0) p = st.idle;
  else if (which == 1) p = st.init;
  else if (which == 2) p = st.scan;
  else p = {st.scanLeftBase, st.scan.bo, st.scan.t};

  if (!armToPose(p, 1500)) return false;
  Serial.println(F(">> Move the arm with the keys, then y."));
  int r = jogLoop(false);
  if (r < 0) return false;
  if (r == 1) {
    Pose3 n = {(int)armBase, (int)armBottom, (int)armTop};
    if (which == 0) st.idle = n;
    else if (which == 1) st.init = n;
    else if (which == 2) st.scan = n;
    else st.scanLeftBase = n.b;
    storeSave();
    Serial.println(F("POSE SAVED."));
  }
  return true;
}

// ======================================================
// CAPTURE N (v15) - teach the camera POSITION FIX of slot N
//   Robot on H CENTER. It drives to slot N (same place as "teach N" / the run).
//   ONLY the arm moves: put the patient in the MIDDLE of the picture, fully visible.
//   y = save: the picture position + size are recorded, then the robot moves a little
//   LEFT and FORWARD (and back) to learn how many pixels = 1 mm.
// ======================================================
static void capLookPrint() {
  int x, y, w, h;
  setAim(IMAGE_CENTER_X, IMAGE_CENTER_Y);
  if (lookPrecise(x, y, w, h, SCAN_TIMEOUT_MS)) {
    Serial.print(F("   camera: centre x="));
    Serial.print(x);
    Serial.print(F(" y="));
    Serial.print(y);
    Serial.print(F("  size "));
    Serial.print(w);
    Serial.print('x');
    Serial.print(h);
    Serial.print(F("   (picture middle = "));
    Serial.print(IMAGE_CENTER_X);
    Serial.print(',');
    Serial.print(IMAGE_CENTER_Y);
    Serial.println(F(")"));
  } else {
    Serial.println(F("   camera: patient NOT seen (or cut by the picture edge) - move the arm"));
  }
}

// arm-only jog: 1 = save, 0 = quit, -1 = stopped
static int capJog() {
  int armStep = 5;
  char line[40];
  Serial.println(F("   ARM: j/l base -/+   i/k bottom +/-   u/o top +/-   z = step 5/2/1"));
  Serial.println(F("   c = camera look (shows centre + size)   y = SAVE   q = quit"));
  capLookPrint();
  while (true) {
    if (!readLine(line, sizeof(line), true)) return -1;
    for (byte i = 0; line[i]; i++) {
      char c = line[i];
      if (c >= 'A' && c <= 'Z') c += 32;
      float nb = armBase, nbo = armBottom, nt = armTop;
      bool mv = false;
      switch (c) {
        case 'y': return 1;
        case 'q': case 'x': Serial.println(F("   quit - NOT saved")); return 0;
        case 'z': armStep = (armStep == 5) ? 2 : (armStep == 2) ? 1 : 5;
                  Serial.print(F("   arm step ")); Serial.println(armStep); break;
        case 'j': nb -= armStep; mv = true; break;
        case 'l': nb += armStep; mv = true; break;
        case 'i': nbo += armStep; mv = true; break;
        case 'k': nbo -= armStep; mv = true; break;
        case 'u': nt += armStep; mv = true; break;
        case 'o': nt -= armStep; mv = true; break;
        case 'c': capLookPrint(); break;
        default: break;
      }
      if (mv && !armMoveTo((int)nb, (int)nbo, (int)nt, 200)) return -1;
      missionAborted = false;
    }
    Serial.print(F("   ARM base="));
    Serial.print((int)armBase);
    Serial.print(F(" bottom="));
    Serial.print((int)armBottom);
    Serial.print(F(" top="));
    Serial.println((int)armTop);
  }
}

// move the robot CAP_CAL_MM with "there", look, come back. vx, vy = px per mm. 1 ok, 0 failed, -1 stop
static int capCal(MoveType there, MoveType back, int x0, int y0, float &vx, float &vy) {
  if (!smallMove(there, CAP_CAL_MM) || !fixHeading()) return -1;
  waitMs(SETTLE_MS);
  int x1, y1, w1, h1;
  setAim(x0, y0);
  bool seen = lookPrecise(x1, y1, w1, h1, SCAN_TIMEOUT_MS);
  if (!smallMove(back, CAP_CAL_MM) || !fixHeading()) return -1;
  if (!seen) return 0;
  vx = (float)(x1 - x0) / CAP_CAL_MM;
  vy = (float)(y1 - y0) / CAP_CAL_MM;
  if (fabs(vx) + fabs(vy) < 0.2) return 0;
  return 1;
}

bool captureTeach(byte s) {
  Serial.println();
  Serial.print(F("=== CAPTURE SLOT "));
  Serial.print(s + 1);
  Serial.println(F(" (camera position fix) ==="));
  Serial.println(F("Patient on the dot, robot on H CENTER facing the hospital."));
  Serial.println(F("TIP: do this right after 'teach N', robot placed exactly the same."));
  if (!teachStart()) return false;
  if (!armToInitial()) return false;
  if (!navToSlot(s)) return false;

  CapData &cd = st.cap[s];
  // always start from the NORMAL camera pose of this slot (scan height, the slot's arm base)
  Pose3 p = {st.slot[s].ok ? st.slot[s].pick.b : slotDefaultPose(s).b, st.scan.bo, st.scan.t};
  Serial.print(F("Arm to the normal camera pose: base="));
  Serial.print(p.b);
  Serial.print(F(" bottom="));
  Serial.print(p.bo);
  Serial.print(F(" top="));
  Serial.println(p.t);
  if (!armToPose(p, ARM_MOVE_MS)) return false;
  waitMs(SETTLE_MS);

  Serial.println(F(">> Move ONLY the ARM until the patient is in the MIDDLE of the picture,"));
  Serial.println(F(">> fully visible (not cut). Check with c. Then y."));
  int r = capJog();
  if (r < 0) return false;

  if (r == 1) {
    waitMs(SETTLE_MS);
    int x0, y0, w0, h0;
    setAim(IMAGE_CENTER_X, IMAGE_CENTER_Y);
    if (!lookPrecise(x0, y0, w0, h0, SCAN_TIMEOUT_MS)) {
      Serial.println(F("!! camera does not see the patient here - NOT saved. Try again."));
    } else {
      Serial.println(F("Calibrating: robot moves a little LEFT and back, then FORWARD and back..."));
      float lx = 0, ly = 0, fx = 0, fy = 0;
      int a = capCal(STRAFE_LEFT, STRAFE_RIGHT, x0, y0, lx, ly);
      if (a < 0) return false;
      int b = capCal(FORWARD, BACKWARD, x0, y0, fx, fy);
      if (b < 0) return false;
      float det = lx * fy - ly * fx;
      if (a != 1 || b != 1 || fabs(det) < 0.05) {
        Serial.println(F("!! calibration FAILED (patient not seen after the move) - NOT saved"));
      } else {
        cd.ok = 1;
        cd.pose = {(int)armBase, (int)armBottom, (int)armTop};
        cd.x = x0; cd.y = y0; cd.w = w0; cd.h = h0;
        cd.lx = lx; cd.ly = ly; cd.fx = fx; cd.fy = fy;
        st.capMagic = CAP_MAGIC;
        storeSave();
        Serial.print(F("CAPTURE "));
        Serial.print(s + 1);
        Serial.print(F(" SAVED: x="));
        Serial.print(x0);
        Serial.print(F(" y="));
        Serial.print(y0);
        Serial.print(F(" size "));
        Serial.print(w0);
        Serial.print('x');
        Serial.print(h0);
        Serial.print(F("   ~"));
        Serial.print(sqrt(lx * lx + ly * ly), 2);
        Serial.println(F(" px per mm"));
      }
    }
  }
  if (!armToInitial()) return false;
  return navReturnNode();
}