// ======================================================
// MISSION 2  -  TEACH & REPLAY   (Arduino Mega)
//
// Serial Monitor: 115200 baud, line ending "Newline" (or "No line ending")
// Type a command and press Enter.  help = all commands.
//
// IDEA: the patient dots are fixed. You TEACH each dot once (robot + arm
// exactly on it) and each zone once. Everything is saved in EEPROM.
// The mission REPLAYS it: camera reads the colour, arm base fine-corrects,
// pick is checked by the camera and repeated if needed.
//
// STOP: STOP button (pin 29) or type x  -  any time.
// Library: Adafruit PWM Servo Driver
// ======================================================
#include "Config.h"

bool missionAborted = false;
bool stepMode = false;
static bool lastStartBtn = HIGH;

// ---------------- STOP ----------------
bool abortRequested() {
  while (Serial.available()) {
    char c = Serial.peek();
    if (c == 'x' || c == 'X') { Serial.read(); missionAborted = true; }
    else if (c == '\n' || c == '\r') Serial.read();
    else break;                       // keep other text for the next command
  }
  if (digitalRead(STOP_BUTTON_PIN) == LOW) missionAborted = true;
  if (missionAborted) stopMotors();
  return missionAborted;
}

// ---------------- LINE INPUT (works with and without line ending) ----------------
static char lb[48];
static byte ll = 0;
static unsigned long lt = 0;

bool readLine(char *buf, byte n, bool blocking) {
  while (true) {
    while (Serial.available()) {
      char c = Serial.read();
      lt = millis();
      if (c == '\n' || c == '\r') {
        if (ll) goto done;
        continue;
      }
      if (ll < sizeof(lb) - 1) lb[ll++] = c;
    }
    if (ll && millis() - lt > 150) goto done;
    if (!blocking) return false;
    if (digitalRead(STOP_BUTTON_PIN) == LOW) { ll = 0; missionAborted = true; stopMotors(); return false; }
#if USE_MPU
    mpuUpdate();
#endif
  }
done:
  lb[ll] = 0;
  strncpy(buf, lb, n);
  buf[n - 1] = 0;
  ll = 0;
  return true;
}

// ---------------- STEP MODE ----------------
bool stepWait() {
  Serial.println(F("   step mode: type n = next   x = stop"));
  char line[16];
  while (true) {
    if (!readLine(line, sizeof(line), true)) return false;
    char c = line[0] | 0x20;
    if (c == 'n') return true;
    if (c == 'x') { missionAborted = true; stopMotors(); return false; }
  }
}

bool mgate(const __FlashStringHelper *name) {
  Serial.print(F("[t="));
  Serial.print((millis() - missionT0) / 1000);
  Serial.print(F(" s] "));
  Serial.println(name);
  if (!stepMode) return true;
  return stepWait();
}

// one drive step: straighten, drive, straighten
bool mdrive(MoveType t, float mm, float slow) {
  if (stepMode) {
    Serial.print(F("NEXT: "));
    Serial.print(moveName(t));
    Serial.print(F(" "));
    Serial.println((int)mm);
    if (!stepWait()) return false;
  }
  if (!fixHeading()) return false;
  if (!drive(t, mm, slow)) return false;
  return fixHeading();
}

// ---------------- HELP ----------------
static void help() {
  Serial.println();
  Serial.println(F("================ MISSION 2 COMMANDS (type + Enter) ================"));
  Serial.println(F("--- SETUP (once, saved) ---"));
  Serial.println(F(" teach N        teach patient slot N (1-12). Robot on H CENTER."));
  Serial.println(F(" capture N      camera POSITION FIX for slot N (arm only) - do it after teach N"));
  Serial.println(F(" capcheck N     test: drive to slot N, camera fix, come back   capoff N / capshow"));
  Serial.println(F(" tz h|p1|p2|r   teach drop zone Hospital / PCC1 / PCC2 / Recovery"));
  Serial.println(F(" pose idle|init|scan|scanl   teach an arm pose"));
  Serial.println(F(" leg1 V / leg2 V / leg3 V    start path (strafe R / fwd / strafe R, mm)"));
  Serial.println(F(" ffwd V / fstrafe V          wheel factors (new = old x asked / measured)"));
  Serial.println(F(" kitspcc 1|0    kits also to PCC1+PCC2 (1) or hospital only (0)"));
  Serial.println(F(" kitset h|p1|p2 REST KICK   set + save one kit servo (e.g. kitset h 90 60)"));
  Serial.println(F(" kitdeg h|p1|p2 DEG          move that kit servo to DEG now (find the angle)"));
  Serial.println(F(" kitshow / kitreset          show kit angles / back to Config.h values"));
  Serial.println(F(" clawset OPEN CLOSE          set + save the claw angles (e.g. clawset 45 180)"));
  Serial.println(F(" verify 1|0     camera pick check ON (repeat if missed) / OFF (trust the claw)"));
  Serial.println(F("--- RUN (robot on H CENTER) ---"));
  Serial.println(F(" pick N         one patient: slot -> pick -> node -> zone -> place -> node"));
  Serial.println(F(" right / left / all   patients 1-6 / 7-12 / 1-12"));
  Serial.println(F(" run            kits + all 12 patients"));
  Serial.println(F(" kits           kit delivery only"));
  Serial.println(F(" look N / lookall     colour only, no pick"));
  Serial.println(F(" goslot N / gozone h|p1|p2|r   drive there and back (check)"));
  Serial.println(F(" tour           MOVEMENT TEST: node -> HOSP -> node -> PCC1 -> node -> PCC2 -> node -> RECOV -> node"));
  Serial.println(F(" tourslots      MOVEMENT TEST: node -> every slot 1-12 -> node"));
  Serial.println(F("--- COMPETITION ---"));
  Serial.println(F(" start  (or START button 22)  5 s -> start zone -> H CENTER -> turn R 90 -> kits -> 12 patients"));
  Serial.println(F(" s              STEP-BY-STEP run from the start zone: stops after each kit / patient, s = next"));
  Serial.println(F(" startkits      KIT TEST: 5 s -> start zone -> H CENTER -> HOSP -> PCC1 -> PCC2 -> H CENTER, stop"));
  Serial.println(F(" node           start zone -> H CENTER (+ turn right 90) only"));
  Serial.println(F(" nodetour       start zone -> H CENTER -> tour of all 4 zones"));
  Serial.println(F(" tr             turn right 90 test"));
  Serial.println(F("--- TESTS ---"));
  Serial.println(F(" cam  live  claw open|close|DEG  kick h|p1|p2|all  arm init|idle|scan|scanl"));
  Serial.println(F(" mfwd N  mback N  mleft N  mright N   (drive test, mm)"));
  Serial.println(F(" gyro  wheels  i2c  show  pre  step  erase yes"));
  Serial.println(F(" x = STOP any time"));
  Serial.println(F("==================================================================="));
}

// ---------------- I2C / PREFLIGHT ----------------
void i2cScan() {
  Serial.println(F("I2C SCAN:"));
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;
    Serial.print(F("   0x"));
    Serial.print(a, HEX);
    if (a == PCA9685_ADDR) Serial.print(F("  arm driver"));
    if (a == 0x62) Serial.print(F("  camera"));
    if (a == 0x68 || a == 0x69) Serial.print(F("  gyro"));
    Serial.println();
  }
}

void preflight() {
  int bad = 0;
  Serial.println();
  Serial.println(F("===== PREFLIGHT ====="));
  Wire.beginTransmission(PCA9685_ADDR);
  if (Wire.endTransmission() == 0) Serial.println(F("OK   arm driver"));
  else { Serial.println(F("FAIL arm driver")); bad++; }
  visionOk = gv2Present();
  if (visionOk) Serial.println(F("OK   camera"));
  else { Serial.println(F("FAIL camera")); bad++; }
  if (mpuOk) Serial.println(F("OK   gyro"));
  else { Serial.println(F("FAIL gyro")); bad++; }
  byte ns = 0, nz = 0;
  for (byte s = 0; s < SLOT_COUNT; s++) if (st.slot[s].ok) ns++;
  for (byte z = 0; z < ZONE_COUNT; z++) if (st.zone[z].ok) nz++;
  Serial.print(ns == SLOT_COUNT ? F("OK   ") : F("WARN "));
  Serial.print(ns);
  Serial.println(F(" / 12 slots taught"));
  Serial.print(nz == ZONE_COUNT ? F("OK   ") : F("WARN "));
  Serial.print(nz);
  Serial.println(F(" / 4 zones taught"));
  if (ns < SLOT_COUNT || nz < ZONE_COUNT) bad++;
  if (digitalRead(STOP_BUTTON_PIN) == LOW) { Serial.println(F("FAIL STOP button pressed")); bad++; }
  if (digitalRead(START_BUTTON_PIN) == LOW) { Serial.println(F("FAIL START button pressed")); bad++; }
  Serial.println(bad ? F("===== FIX THE POINTS ABOVE =====") : F("===== ALL GOOD - READY ====="));
}

// ---------------- helpers ----------------
static int slotArg(const char *a) {
  int n = atoi(a);
  if (n < 1 || n > SLOT_COUNT) { Serial.println(F("slot must be 1 - 12")); return -1; }
  return n - 1;
}

static int zoneArg(const char *a) {
  if (!strcmp(a, "h"))  return ZONE_HOSPITAL;
  if (!strcmp(a, "p1")) return ZONE_PCC1;
  if (!strcmp(a, "p2")) return ZONE_PCC2;
  if (!strcmp(a, "r"))  return ZONE_RECOVERY;
  Serial.println(F("zone: h  p1  p2  r"));
  return -1;
}

// h -> 0 (hospital)  p1 -> 1  p2 -> 2   (= Zone number = kit index)
static int kitZoneArg(const char *a) {
  if (!strcmp(a, "h"))  return ZONE_HOSPITAL;
  if (!strcmp(a, "p1")) return ZONE_PCC1;
  if (!strcmp(a, "p2")) return ZONE_PCC2;
  return -1;
}

static bool nodeBegin() {
  missionReset();
  setSuction(false);
  Serial.println(F("Robot on H CENTER, facing the hospital."));
  return gyroStart(0);
}

static void finish(bool ok) {
  stopMotors();
  if (!ok) {
    setSuction(false);
    Serial.println(missionAborted ? F("!!! STOPPED") : F("!!! A STEP FAILED"));
  } else {
    Serial.println(F("=== DONE ==="));
  }
  printSummary();
  missionAborted = false;
}

static void testDrive(MoveType t, const char *a) {
  float mm = atof(a);
  if (mm <= 0) mm = 300;
  missionReset();
#if USE_MPU
  mpuUpdate();
  headingRef = yawDeg;
#endif
  drive(t, mm, 40);
  fixHeading();
  Serial.print(F("Measure it. If it is wrong: new factor = old x "));
  Serial.print(mm, 0);
  Serial.println(F(" / measured"));
}

static void cameraLive() {
  Serial.println(F("CAMERA LIVE 20 s  (x = stop)"));
  unsigned long t0 = millis();
  while (millis() - t0 < 20000) {
    if (abortRequested()) break;
    if (gv2Invoke() != 0) { Serial.println(F("no answer")); continue; }
    if (!boxCount) { Serial.println(F("nothing")); continue; }
    for (int i = 0; i < boxCount; i++) {
      Serial.print(colorName(classToColor(boxes[i].target)));
      Serial.print(F(" s="));
      Serial.print(boxes[i].score);
      Serial.print(F(" x="));
      Serial.print(boxes[i].x);
      Serial.print(F(" y="));
      Serial.print(boxes[i].y);
      Serial.print(F(" w="));
      Serial.print(boxes[i].w);
      Serial.print(F(" h="));
      Serial.print(boxes[i].h);
      if (boxCut(boxes[i])) Serial.print(F(" CUT"));
      Serial.print(F("   "));
    }
    Serial.println();
  }
  missionAborted = false;
}

// ---------------- CAPCHECK RESULT (printed LAST, easy to copy) ----------------
extern byte capResult;
extern float capLastL, capLastF;
static void capCheckReport(byte s) {
  Serial.println(F("=========== CAPCHECK RESULT ==========="));
  Serial.println(F(BUILD_NAME));
  Serial.print(F("slot "));
  Serial.print(s + 1);
  Serial.print(F(": capture saved = "));
  Serial.println(st.cap[s].ok ? F("YES") : F("NO"));
  if (st.cap[s].ok) {
    Serial.print(F("  captured pix "));
    Serial.print(st.cap[s].x); Serial.print(','); Serial.print(st.cap[s].y);
    Serial.print(F("  size ")); Serial.print(st.cap[s].w); Serial.print('x'); Serial.print(st.cap[s].h);
    Serial.print(F("  arm ")); Serial.print(st.cap[s].pose.b); Serial.print('/');
    Serial.print(st.cap[s].pose.bo); Serial.print('/'); Serial.println(st.cap[s].pose.t);
  }
  Serial.print(F("result: "));
  switch (capResult) {
    case 1: Serial.println(F("NO CAPTURE SAVED -> run: capture N  (wait for 'CAPTURE N SAVED')")); break;
    case 2: Serial.println(F("BAD CALIBRATION -> run: capture N again")); break;
    case 3: Serial.println(F("PATIENT NOT SEEN from the capture pose -> patient on the dot? run capture N again")); break;
    case 4: Serial.println(F("PATIENT SIZE DIFFERENT -> arm pose wrong, run capture N again")); break;
    case 5: Serial.println(F("ERROR TOO BIG (>40 mm) -> robot placed very differently on H CENTER")); break;
    case 6: Serial.println(F("FIXED - robot corrected, map updated")); break;
    case 7: Serial.println(F("MOVED but did not reach 1.5 mm")); break;
    default: Serial.println(F("capture fix did NOT run (CAPTURE_ENABLE 0?)")); break;
  }
  if (capResult >= 5) {
    Serial.print(F("last error: robot must go "));
    Serial.print(capLastL, 1);
    Serial.print(F(" mm left, "));
    Serial.print(capLastF, 1);
    Serial.println(F(" mm forward"));
  }
  Serial.println(F("======================================="));
}

// ---------------- COMMANDS ----------------
static void handle(char *line) {
  for (char *p = line; *p; p++) if (*p >= 'A' && *p <= 'Z') *p += 32;
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *a = strtok(NULL, " \t");
  if (!a) a = (char *)"";
  int s, z;
  bool ok = true;

  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) help();
  else if (!strcmp(cmd, "show")) storeShow();
  else if (!strcmp(cmd, "pre")) preflight();
  else if (!strcmp(cmd, "i2c")) i2cScan();
  else if (!strcmp(cmd, "step")) { stepMode = !stepMode; Serial.println(stepMode ? F("STEP MODE ON (n = next)") : F("STEP MODE OFF")); }

  // ---- teach ----
  else if (!strcmp(cmd, "teach")) { if ((s = slotArg(a)) >= 0) { ok = teachSlot(s); finish(ok); } }
  else if (!strcmp(cmd, "capture")) { if ((s = slotArg(a)) >= 0) { ok = captureTeach(s); finish(ok); } }
  else if (!strcmp(cmd, "capcheck")) {
    // drive to slot N, do the camera position fix only (no pick), come back
    if ((s = slotArg(a)) >= 0) {
      ok = nodeBegin() && armToInitial() && navToSlot(s) && captureFix(s) && waitMs(1000) && armToInitial() && navReturnNode();
      finish(ok);
      capCheckReport(s);
    }
  }
  else if (!strcmp(cmd, "capoff")) {
    if ((s = slotArg(a)) >= 0) { st.cap[s].ok = 0; storeSave(); Serial.println(F("capture deleted for this slot")); }
  }
  else if (!strcmp(cmd, "capshow")) capShow();
  else if (!strcmp(cmd, "tz"))    { if ((z = zoneArg(a)) >= 0) { ok = teachZone((Zone)z); finish(ok); } }
  else if (!strcmp(cmd, "pose")) {
    int w = !strcmp(a, "idle") ? 0 : !strcmp(a, "init") ? 1 : !strcmp(a, "scan") ? 2 : !strcmp(a, "scanl") ? 3 : -1;
    if (w < 0) Serial.println(F("pose idle | init | scan | scanl"));
    else teachPose(w);
    missionAborted = false;
  }
  else if (!strcmp(cmd, "leg1")) { st.leg1 = atof(a); storeSave(); storeShow(); }
  else if (!strcmp(cmd, "leg2")) { st.leg2 = atof(a); storeSave(); storeShow(); }
  else if (!strcmp(cmd, "leg3")) { st.leg3 = atof(a); storeSave(); storeShow(); }
  else if (!strcmp(cmd, "ffwd"))    { float f = atof(a); if (f > 0.3 && f < 3) { st.fwdF = f; storeSave(); } storeShow(); }
  else if (!strcmp(cmd, "fback"))   { float f = atof(a); if (f > 0.3 && f < 3) { st.backF = f; storeSave(); } storeShow(); }
  else if (!strcmp(cmd, "fstrafe")) { float f = atof(a); if (f > 0.3 && f < 3) { st.strafeF = f; storeSave(); } storeShow(); }
  else if (!strcmp(cmd, "kitspcc")) { st.kitsPCC = atoi(a) ? 1 : 0; storeSave(); storeShow(); }
  else if (!strcmp(cmd, "kitset")) {
    // kitset h 90 60   = zone, REST degrees, KICK degrees
    int kz = kitZoneArg(a);
    char *a2 = strtok(NULL, " \t");
    char *a3 = strtok(NULL, " \t");
    if (kz < 0 || !a2 || !a3) Serial.println(F("type:  kitset h|p1|p2 REST KICK    e.g.  kitset h 90 60"));
    else {
      int r = atoi(a2), k = atoi(a3);
      if (r < 0 || r > 180 || k < 0 || k > 180) Serial.println(F("angles must be 0 - 180"));
      else {
        st.kitRest[kz] = r; st.kitKick[kz] = k; st.kitOk = KIT_OK_MAGIC;
        storeSave();
        kitMoveDeg((Zone)kz, r);
        Serial.println(F("KIT SAVED (servo now at REST). Test with: kick h|p1|p2"));
        kitShow();
      }
    }
  }
  else if (!strcmp(cmd, "kitdeg")) {
    // kitdeg p1 70  = move that servo to 70 degrees now (nothing saved)
    int kz = kitZoneArg(a);
    char *a2 = strtok(NULL, " \t");
    if (kz < 0 || !a2) Serial.println(F("type:  kitdeg h|p1|p2 DEG    e.g.  kitdeg p1 70"));
    else {
      int d = constrain(atoi(a2), 0, 180);
      kitMoveDeg((Zone)kz, d);
      Serial.print(F("kit servo ")); Serial.print(zoneName((Zone)kz));
      Serial.print(F(" -> ")); Serial.print(d); Serial.println(F(" deg (not saved - use kitset)"));
    }
  }
  else if (!strcmp(cmd, "kitshow")) kitShow();
  else if (!strcmp(cmd, "clawset")) {
    // clawset 45 180  = OPEN degrees, CLOSE degrees  (saved)
    char *a2 = strtok(NULL, " \t");
    if (!*a || !a2) Serial.println(F("type:  clawset OPEN CLOSE    e.g.  clawset 45 180"));
    else {
      int o = atoi(a), c = atoi(a2);
      if (o < 0 || o > 180 || c < 0 || c > 180) Serial.println(F("angles must be 0 - 180"));
      else {
        st.clawOpen = o; st.clawClose = c; st.clawOk = CLAW_OK_MAGIC;
        storeSave();
        setSuction(false);
        Serial.println(F("CLAW SAVED (claw now OPEN). Test with: claw close / claw open"));
        clawShow();
      }
    }
  }
  else if (!strcmp(cmd, "verify")) {
    // verify 1 = camera checks each pick (repeats if missed)   verify 0 = trust the claw
    if (*a) { st.pickVerify = atoi(a) ? 1 : 0; st.clawOk = CLAW_OK_MAGIC; storeSave(); }
    clawShow();
  }
  else if (!strcmp(cmd, "kitreset")) { kitDefaults(); storeSave(); kitsHoldAll(); kitShow(); }
  else if (!strcmp(cmd, "erase")) {
    if (!strcmp(a, "yes")) { storeDefaults(); storeSave(); Serial.println(F("ALL TAUGHT DATA ERASED")); }
    else Serial.println(F("type:  erase yes"));
  }

  // ---- run from H CENTER ----
  else if (!strcmp(cmd, "pick")) { if ((s = slotArg(a)) >= 0) { ok = nodeBegin() && processSlot(s); finish(ok); } }
  else if (!strcmp(cmd, "right")) { ok = nodeBegin(); for (byte i = 0; ok && i < 6; i++) ok = processSlot(i); finish(ok); }
  else if (!strcmp(cmd, "left"))  { ok = nodeBegin(); for (byte i = 6; ok && i < 12; i++) ok = processSlot(i); finish(ok); }
  else if (!strcmp(cmd, "all"))   { ok = nodeBegin(); for (byte i = 0; ok && i < 12; i++) ok = processSlot(i); finish(ok); }
  else if (!strcmp(cmd, "run"))   { ok = nodeBegin() && runFromNode(true); finish(ok); }
  else if (!strcmp(cmd, "kits"))  { ok = nodeBegin() && deliverKits(); finish(ok); }
  else if (!strcmp(cmd, "look"))  {
    if ((s = slotArg(a)) >= 0) {
      PatientColor c;
      ok = nodeBegin() && armToInitial() && navToSlot(s) && lookSlot(s, c);
      if (ok) { Serial.print(F("SLOT ")); Serial.print(s + 1); Serial.print(F(": ")); Serial.println(colorName(c)); }
      ok = ok && armToInitial() && navReturnNode();
      finish(ok);
    }
  }
  else if (!strcmp(cmd, "lookall")) { if (nodeBegin()) lookAll(); finish(!missionAborted); }
  else if (!strcmp(cmd, "goslot")) {
    if ((s = slotArg(a)) >= 0) { ok = nodeBegin() && armToInitial() && navToSlot(s) && waitMs(1500) && navReturnNode(); finish(ok); }
  }
  else if (!strcmp(cmd, "gozone")) {
    if ((z = zoneArg(a)) >= 0) { ok = nodeBegin() && armToInitial() && navToZone((Zone)z, 0) && waitMs(1500) && navReturnNode(); finish(ok); }
  }

  // ---- competition ----
  else if (!strcmp(cmd, "start")) { ok = runMission(); finish(ok); }
  else if (!strcmp(cmd, "s")) {
    // STEP-BY-STEP competition run: robot in the START ZONE. Stops after every job, s = next
    Serial.println(F("S-RUN: full run, it waits for  s  after every job  (x = stop)"));
    sRun = true;
    ok = runMission();
    sRun = false;
    finish(ok);
  }
  else if (!strcmp(cmd, "node"))  { missionReset(); ok = armToInitial() && gyroStart(0) && navStartToNode(); finish(ok); }
  else if (!strcmp(cmd, "tour"))  { ok = nodeBegin() && zoneTour(); finish(ok); }
  else if (!strcmp(cmd, "tourslots")) { ok = nodeBegin() && slotTour(); finish(ok); }
  else if (!strcmp(cmd, "startkits")) { ok = startKitsOnly(); finish(ok); }
  else if (!strcmp(cmd, "nodetour")) { missionReset(); ok = armToInitial() && gyroStart(0) && navStartToNode() && zoneTour(); finish(ok); }
  else if (!strcmp(cmd, "tr")) { missionReset(); ok = gyroStart(0) && mdrive(ROTATE_RIGHT, TURN_90_WHEEL_MM, 60); finish(ok); }

  // ---- tests ----
  else if (!strcmp(cmd, "cam"))  cameraTest();
  else if (!strcmp(cmd, "live")) cameraLive();
  else if (!strcmp(cmd, "suck")) setSuction(!suctionActive);
  else if (!strcmp(cmd, "claw")) {
    // claw open / claw close / claw 120 (any angle now, not saved)
    if (!strcmp(a, "open")) setSuction(false);
    else if (!strcmp(a, "close")) setSuction(true);
    else if (*a) {
      int d = constrain(atoi(a), 0, 180);
      clawMoveDeg(d);
      Serial.print(F("claw -> ")); Serial.print(d); Serial.println(F(" deg (not saved - use clawset OPEN CLOSE)"));
    }
    else Serial.println(F("type:  claw open | claw close | claw DEG"));
  }
  else if (!strcmp(cmd, "kick")) {
    if (!strcmp(a, "h")) kitRelease(ZONE_HOSPITAL);
    else if (!strcmp(a, "p1")) kitRelease(ZONE_PCC1);
    else if (!strcmp(a, "p2")) kitRelease(ZONE_PCC2);
    else { kitRelease(ZONE_HOSPITAL); kitRelease(ZONE_PCC1); kitRelease(ZONE_PCC2); }
    missionAborted = false;
  }
  else if (!strcmp(cmd, "arm")) {
    if (!strcmp(a, "idle")) armToIdle();
    else if (!strcmp(a, "scan")) armScanAt(st.scan.b);
    else if (!strcmp(a, "scanl")) armScanAt(st.scanLeftBase);
    else armToInitial();
    missionAborted = false;
  }
  else if (!strcmp(cmd, "mfwd"))   testDrive(FORWARD, a);
  else if (!strcmp(cmd, "mback"))  testDrive(BACKWARD, a);
  else if (!strcmp(cmd, "mleft"))  testDrive(STRAFE_LEFT, a);
  else if (!strcmp(cmd, "mright")) testDrive(STRAFE_RIGHT, a);
  else if (!strcmp(cmd, "gyro"))   mpuTest();
  else if (!strcmp(cmd, "wheels")) wheelTest();
  else if (!strcmp(cmd, "x")) { stopMotors(); setSuction(false); }
  else { Serial.print(F("unknown: ")); Serial.print(cmd); Serial.println(F("   (type help)")); }

  stopMotors();
  missionAborted = false;
}

// ---------------- SETUP / LOOP ----------------
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

  Serial.println();
  Serial.println(F(BUILD_NAME));

  storeLoad();
  speedLoad();

  Wire.begin();
  Wire.setWireTimeout(25000, true);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(100);
  setSuction(false);
  kitsHoldAll();

#if USE_MPU
  Serial.print(F("Gyro... "));
  if (mpuBegin()) { Serial.println(F("OK - keep the robot still")); mpuCalibrate(); }
  else Serial.println(F("NOT FOUND"));
#endif

  armBase = st.idle.b; armBottom = st.idle.bo; armTop = st.idle.t;
  armApply();

  Serial.print(F("Camera... "));
  for (int i = 0; i < 20 && !visionOk; i++) { visionOk = gv2Present(); if (!visionOk) delay(300); }
  Serial.println(visionOk ? F("OK") : F("NOT FOUND"));

  armToInitial();
  missionAborted = false;
  preflight();
  help();
  Serial.println(F("READY. Type a command (help)."));
}

void loop() {
  char line[48];
  if (readLine(line, sizeof(line), false)) {
    Serial.print(F("> "));
    Serial.println(line);
    handle(line);
  }

  bool b = digitalRead(START_BUTTON_PIN);
  if (lastStartBtn == HIGH && b == LOW) {
    delay(30);
    
    if (digitalRead(START_BUTTON_PIN) == LOW) finish(runMission());
  }
  lastStartBtn = b;

#if USE_MPU
  mpuUpdate();
#endif
}