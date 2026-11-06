// ======================================================
// Store.ino  -  EVERYTHING YOU TEACH IS SAVED HERE (EEPROM)
// It survives power off. Command  erase yes  = back to the Config.h values.
// ======================================================
#include "Config.h"

Store st;

bool slotIsRight(byte s) { return s < 6; }

// First guess of a slot's pick pose: arm to that side, default reach
Pose3 slotDefaultPose(byte s) {
  Pose3 p;
  p.b = slotIsRight(s) ? st.scan.b : st.scanLeftBase;
  p.bo = PICK_BOTTOM;
  p.t = PICK_TOP;
  return p;
}

// kit servo angles from Config.h  ([0] hospital  [1] PCC1  [2] PCC2)
void kitDefaults() {
  st.kitRest[0] = KIT_HOSP_REST;  st.kitKick[0] = KIT_HOSP_KICK;
  st.kitRest[1] = KIT_PCC1_REST;  st.kitKick[1] = KIT_PCC1_KICK;
  st.kitRest[2] = KIT_PCC2_REST;  st.kitKick[2] = KIT_PCC2_KICK;
  st.kitOk = KIT_OK_MAGIC;
}

// claw angles from Config.h, pick check ON
void clawDefaults() {
  st.clawOpen = CLAW_OPEN_DEG;
  st.clawClose = CLAW_CLOSE_DEG;
  st.pickVerify = 1;
  st.clawOk = CLAW_OK_MAGIC;
}

void storeDefaults() {
  memset(&st, 0, sizeof(st));
  st.magic = STORE_MAGIC;
  st.idle = {IDLE_BASE, IDLE_BOTTOM, IDLE_TOP};
  st.init = {INITIAL_BASE, INITIAL_BOTTOM, INITIAL_TOP};
  st.scan = {SCAN_BASE, SCAN_BOTTOM, SCAN_TOP};
  st.scanLeftBase = SCAN_LEFT_BASE;
  for (byte s = 0; s < SLOT_COUNT; s++) st.slot[s].pick = slotDefaultPose(s);
  for (byte z = 0; z < ZONE_COUNT; z++) st.zone[z].place = {PLACE_BASE, PLACE_BOTTOM, PLACE_TOP};
  st.fwdF = DEF_FORWARD_FACTOR;
  st.strafeF = DEF_STRAFE_FACTOR;
  st.backF = DEF_FORWARD_FACTOR;
  st.leg1 = DEF_LEG1;
  st.leg2 = DEF_LEG2;
  st.leg3 = DEF_LEG3;
  st.kitsPCC = 1;
  kitDefaults();
  clawDefaults();
}

void storeSave() {
  st.magic = STORE_MAGIC;
  EEPROM.put(0, st);
}

void storeLoad() {
  EEPROM.get(0, st);
  if (st.magic != STORE_MAGIC || st.fwdF < 0.3 || st.fwdF > 3.0) {
    Serial.println(F("EEPROM: nothing taught yet -> start values"));
    storeDefaults();
    storeSave();
  }
  if (isnan(st.backF) || st.backF < 0.3 || st.backF > 3.0) { st.backF = st.fwdF; storeSave(); }
  // v6 fields: data saved by v5 has nothing here -> "not calibrated"
  bool fix = false;
  for (byte s = 0; s < SLOT_COUNT; s++) {
    if (st.strafeCal[s] != 1 || isnan(st.sx[s]) || isnan(st.sy[s]) ||
        fabs(st.sx[s]) > 50 || fabs(st.sy[s]) > 50) {
      if (st.strafeCal[s] != 0) fix = true;
      st.strafeCal[s] = 0; st.sx[s] = 0; st.sy[s] = 0;
    }
    // v11 field: forward/back calibration
    if (st.fwdCal[s] != 1 || isnan(st.fwx[s]) || isnan(st.fwy[s]) ||
        fabs(st.fwx[s]) > 50 || fabs(st.fwy[s]) > 50) {
      if (st.fwdCal[s] != 0) fix = true;
      st.fwdCal[s] = 0; st.fwx[s] = 0; st.fwy[s] = 0;
    }
  }
  // v9 fields: kit angles never saved yet -> take the Config.h values
  bool kitBad = (st.kitOk != KIT_OK_MAGIC);
  for (byte k = 0; k < KIT_COUNT; k++)
    if (st.kitRest[k] > 180 || st.kitKick[k] > 180) kitBad = true;
  if (kitBad) {
    Serial.println(F("EEPROM: kit angles -> Config.h values"));
    kitDefaults();
    fix = true;
  }
  // v10 fields: claw angles never saved yet -> take the Config.h values
  if (st.clawOk != CLAW_OK_MAGIC || st.clawOpen > 180 || st.clawClose > 180 || st.pickVerify > 1) {
    Serial.println(F("EEPROM: claw angles -> Config.h values"));
    clawDefaults();
    fix = true;
  }
  // v15 field: captures never saved yet -> none
  if (st.capMagic != CAP_MAGIC) {
    memset(st.cap, 0, sizeof(st.cap));
    st.capMagic = CAP_MAGIC;
    fix = true;
  }
  for (byte s = 0; s < SLOT_COUNT; s++)
    if (st.cap[s].ok > 1 || isnan(st.cap[s].lx) || isnan(st.cap[s].fx)) { memset(&st.cap[s], 0, sizeof(CapData)); fix = true; }
  if (fix) storeSave();
}

void capShow() {
  Serial.println(F("--- CAPTURE (camera position fix) ---"));
  byte n = 0;
  for (byte s = 0; s < SLOT_COUNT; s++) if (st.cap[s].ok) n++;
  if (!n) Serial.println(F("  none saved yet (type: capture N)"));
  for (byte s = 0; s < SLOT_COUNT; s++) {
    if (!st.cap[s].ok) continue;
    Serial.print(F("  slot "));
    Serial.print(s + 1);
    Serial.print(F("  pix "));
    Serial.print(st.cap[s].x);
    Serial.print(',');
    Serial.print(st.cap[s].y);
    Serial.print(F("  size "));
    Serial.print(st.cap[s].w);
    Serial.print('x');
    Serial.print(st.cap[s].h);
    Serial.print(F("  arm "));
    Serial.print(st.cap[s].pose.b);
    Serial.print('/');
    Serial.print(st.cap[s].pose.bo);
    Serial.print('/');
    Serial.println(st.cap[s].pose.t);
  }
}

static void pPose(const Pose3 &p) {
  Serial.print(F("base="));
  Serial.print(p.b);
  Serial.print(F(" bottom="));
  Serial.print(p.bo);
  Serial.print(F(" top="));
  Serial.print(p.t);
}

void kitShow() {
  const char *names[KIT_COUNT] = {"HOSPITAL (h) ", "PCC1     (p1)", "PCC2     (p2)"};
  const Zone zs[KIT_COUNT] = {ZONE_HOSPITAL, ZONE_PCC1, ZONE_PCC2};
  Serial.println(F("--- KIT SERVOS ---"));
  for (byte k = 0; k < KIT_COUNT; k++) {
    Serial.print(F("  "));
    Serial.print(names[k]);
    Serial.print(F("  channel "));
    Serial.print(kitChannel(zs[k]));
    Serial.print(F("  rest "));
    Serial.print(st.kitRest[k]);
    Serial.print(F("  kick "));
    Serial.println(st.kitKick[k]);
  }
}

void clawShow() {
  Serial.print(F("--- GRIPPER: "));
#if GRIPPER_CLAW
  Serial.print(F("CLAW on channel "));
  Serial.print(CLAW_CHANNEL);
  Serial.print(F("   open "));
  Serial.print(st.clawOpen);
  Serial.print(F("   close "));
  Serial.print(st.clawClose);
#else
  Serial.print(F("SUCTION pump on channel "));
  Serial.print(SUCTION_CHANNEL);
#endif
  Serial.print(F("   pick check (verify): "));
  Serial.println(st.pickVerify ? F("ON") : F("OFF"));
}

void storeShow() {
  Serial.println();
  Serial.println(F("=============== SAVED DATA ==============="));
  Serial.print(F("IDLE    ")); pPose(st.idle); Serial.println();
  Serial.print(F("INITIAL ")); pPose(st.init); Serial.println();
  Serial.print(F("SCAN R  ")); pPose(st.scan); Serial.println();
  Serial.print(F("SCAN L  base=")); Serial.println(st.scanLeftBase);
  Serial.print(F("factors: fwd=")); Serial.print(st.fwdF, 3);
  Serial.print(F("  strafe=")); Serial.println(st.strafeF, 3);
  Serial.print(F("back factor: ")); Serial.println(st.backF, 3);
  Serial.print(F("start legs: right ")); Serial.print(st.leg1, 1);
  Serial.print(F("  fwd ")); Serial.print(st.leg2, 1);
  Serial.print(F("  right ")); Serial.println(st.leg3, 1);
  Serial.print(F("camera base cal: "));
  if (st.baseCal) { Serial.print(st.bx); Serial.print(F(", ")); Serial.println(st.by); }
  else Serial.println(F("not yet (done by the first teach)"));
  Serial.print(F("kits to PCC: ")); Serial.println(st.kitsPCC ? F("yes") : F("no (hospital only)"));
  kitShow();
  capShow();
  clawShow();

  Serial.println(F("--- PATIENT SLOTS (1-6 right, 7-12 left) ---"));
  for (byte s = 0; s < SLOT_COUNT; s++) {
    Serial.print(F("  slot "));
    if (s < 9) Serial.print(' ');
    Serial.print(s + 1);
    Serial.print(st.slot[s].ok ? F("  TAUGHT   ") : F("  -------  "));
    if (st.slot[s].ok) {
      Serial.print(F("move "));
      Serial.print(st.slot[s].dx, 0);
      Serial.print(',');
      Serial.print(st.slot[s].dy, 0);
      Serial.print(F(" mm  "));
      pPose(st.slot[s].pick);
      if (st.slot[s].hasPix) {
        Serial.print(F("  pix "));
        Serial.print(st.slot[s].px);
        Serial.print(',');
        Serial.print(st.slot[s].py);
      }
      Serial.print(st.strafeCal[s] ? F("  cal: L/R OK") : F("  cal: L/R --"));
      Serial.print(st.fwdCal[s] ? F("  F/B OK") : F("  F/B --"));
    }
    Serial.println();
  }
  Serial.println(F("--- ZONES ---"));
  for (byte z = 0; z < ZONE_COUNT; z++) {
    Serial.print(F("  "));
    Serial.print(zoneName((Zone)z));
    Serial.print(st.zone[z].ok ? F("  TAUGHT   ") : F("  -------  "));
    if (st.zone[z].ok) {
      Serial.print(F("move "));
      Serial.print(st.zone[z].dx, 0);
      Serial.print(',');
      Serial.print(st.zone[z].dy, 0);
      Serial.print(F(" mm  "));
      pPose(st.zone[z].place);
    }
    Serial.println();
  }
  Serial.println(F("=========================================="));
}