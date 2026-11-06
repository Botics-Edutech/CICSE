// ======================================================
// Nav.ino  -  MAP + DRIVING FROM / TO THE H CENTER (node)
//
// (0,0) = H CENTER. +X = forward (hospital), +Y = left (PCC1).
// Out: X first, then Y.   Back: Y first, then X (same path).
//
// SLOTS (the robot stops BESIDE the dot, the arm reaches sideways):
//   1 2 3  = RIGHT, hospital row (1 = nearest)     arm RIGHT
//   4 5 6  = RIGHT, start row    (4 = nearest)     arm RIGHT
//   7 8 9  = LEFT,  hospital row (7 = nearest)     arm LEFT
//  10 11 12= LEFT,  start row   (10 = nearest)     arm LEFT
// Each slot / zone adds its TAUGHT offset (teach N / tz ...).
// ======================================================
#include "Config.h"

float navX = 0, navY = 0;
static int8_t navLastZone = -1;   // v18b: zone the robot is at (-1 = not at a zone) -> return assist

// ---- START PATH CORRECTION (from the video, v10b) ----
// The saved legs (leg1 / leg2 / leg3, see "show") are used PLUS these corrections:
//   1st strafe RIGHT was ~5 cm too long -> the robot drove over the first patient column
//   -> make leg1 shorter and leg3 longer by the same amount (robot still ends on H CENTER)
// Fine-tune here (mm), upload again:
//   robot still too close to the patients after the 1st strafe -> LEG1_ADJ more negative (-60, -70)
//   2nd strafe stops SHORT of the H centre  -> LEG3_ADJ bigger   (e.g. +65)
//   2nd strafe goes PAST the H centre       -> LEG3_ADJ smaller  (e.g. +35)
#ifndef START_LEG1_ADJ
#define START_LEG1_ADJ   -50.0
#endif
#ifndef START_LEG2_ADJ
#define START_LEG2_ADJ     0.0
#endif
#ifndef START_LEG3_ADJ
#define START_LEG3_ADJ   +50.0
#endif

bool navRelX(float mm) {
  if (fabs(mm) <= NAV_TOL_MM) return true;
  return mdrive(mm > 0 ? FORWARD : BACKWARD, fabs(mm), SZ(fabs(mm)));
}

bool navRelY(float mm) {
  if (fabs(mm) <= NAV_TOL_MM) return true;
  return mdrive(mm > 0 ? STRAFE_LEFT : STRAFE_RIGHT, fabs(mm), SZ(fabs(mm)));
}

bool navGoTo(float x, float y, const __FlashStringHelper *name) {
  Serial.print(F("NAV -> "));
  Serial.print(name);
  Serial.print(F(" ("));
  Serial.print(x, 0);
  Serial.print(F(", "));
  Serial.print(y, 0);
  Serial.println(F(")"));
  if (!navRelX(x - navX)) return false;
  navX = x;
  if (!navRelY(y - navY)) return false;
  navY = y;
  return true;
}

// v18b: extra mm for the way back from a zone (see RET_..._ADJ in Config.h)
static void returnAdj(int8_t z, float &side, float &fb) {
  side = 0; fb = 0;
  switch (z) {
    case ZONE_HOSPITAL: side = RET_HOSP_SIDE_ADJ;  fb = RET_HOSP_FB_ADJ;  break;
    case ZONE_PCC1:     side = RET_PCC1_SIDE_ADJ;  fb = RET_PCC1_FB_ADJ;  break;
    case ZONE_PCC2:     side = RET_PCC2_SIDE_ADJ;  fb = RET_PCC2_FB_ADJ;  break;
    case ZONE_RECOVERY: side = RET_RECOV_SIDE_ADJ; fb = RET_RECOV_FB_ADJ; break;
    default: break;
  }
}

bool navReturnNode() {
  Serial.println(F("NAV -> H CENTER"));
  float side, fb;
  returnAdj(navLastZone, side, fb);
  navLastZone = -1;
  // the extra mm go in the SAME direction as the leg (only if the leg is driven)
  float dy = -navY, dx = -navX;
  if (fabs(dy) > NAV_TOL_MM && side != 0) {
    dy += (dy > 0 ? side : -side);
    Serial.print(F("   return assist sideways "));
    Serial.print(side, 0);
    Serial.println(F(" mm"));
  }
  if (fabs(dx) > NAV_TOL_MM && fb != 0) {
    dx += (dx > 0 ? fb : -fb);
    Serial.print(F("   return assist fwd/back "));
    Serial.print(fb, 0);
    Serial.println(F(" mm"));
  }
  if (!navRelY(dy)) return false;
  navY = 0;
  if (!navRelX(dx)) return false;
  navX = 0;
  return true;
}

// start zone -> H CENTER   (legs: commands leg1 / leg2 / leg3)
// strafe RIGHT leg1 -> FORWARD leg2 -> strafe RIGHT leg3 -> TURN RIGHT 90 (now facing the hospital)
bool navStartToNode() {
  float l1 = st.leg1 + START_LEG1_ADJ;
  float l2 = st.leg2 + START_LEG2_ADJ;
  float l3 = st.leg3 + START_LEG3_ADJ;
  Serial.println(F("=== START ZONE -> H CENTER ==="));
  Serial.print(F("   legs used: strafe R "));
  Serial.print(l1, 1);
  Serial.print(F("  fwd "));
  Serial.print(l2, 1);
  Serial.print(F("  strafe R "));
  Serial.println(l3, 1);
  if (!mgate(F("1) strafe RIGHT out of the start zone"))) return false;
  if (l1 > NAV_TOL_MM && !mdrive(STRAFE_RIGHT, l1, SZ(l1))) return false;
  if (!mgate(F("2) FORWARD"))) return false;
  if (l2 > NAV_TOL_MM && !mdrive(FORWARD, l2, SZ(l2))) return false;
  if (!mgate(F("3) strafe RIGHT onto H CENTER"))) return false;
  if (l3 > NAV_TOL_MM && !mdrive(STRAFE_RIGHT, l3, SZ(l3))) return false;
#if START_TURN_RIGHT
  if (!mgate(F("4) TURN RIGHT 90 - face the hospital"))) return false;
  if (!mdrive(ROTATE_RIGHT, TURN_90_WHEEL_MM, 60)) return false;
#endif
  navX = 0;
  navY = 0;
  Serial.println(F("AT H CENTER"));
  return true;
}

// ---- slots ----
#ifndef SLOT_HOSP_ADJ
#define SLOT_HOSP_ADJ {0,0,0,0,0,0,0,0,0,0,0,0}
#endif
#ifndef SLOT_BACK_ADJ
#define SLOT_BACK_ADJ {0,0,0,0,0,0,0,0,0,0,0,0}
#endif
static void slotNominal(byte s, float &x, float &y) {
  bool right = slotIsRight(s);
  byte i = s % 6;
  x = (i < 3) ? ROW_HOSP_X : ROW_START_X;
  byte k = i % 3;
  float dotY;
  if (right) dotY = (k == 0) ? RIGHT_Y1 : (k == 1) ? RIGHT_Y2 : RIGHT_Y3;
  else       dotY = (k == 0) ? LEFT_Y1  : (k == 1) ? LEFT_Y2  : LEFT_Y3;
  y = dotY + (right ? PICK_REACH_MM : -PICK_REACH_MM);
  static const float slotBack[12] = SLOT_BACK_ADJ;   // v18c: per-slot "stop further back"
  x -= slotBack[s];
  static const float slotHosp[12] = SLOT_HOSP_ADJ;    // v18i: + = towards the hospital
  x += slotHosp[s];
}

void slotStop(byte s, float &x, float &y) {
  slotNominal(s, x, y);
  if (st.slot[s].ok) { x += st.slot[s].dx; y += st.slot[s].dy; }
}

bool navToSlot(byte s) {
  float x, y;
  slotStop(s, x, y);
  Serial.print(F("SLOT "));
  Serial.println(s + 1);
  navLastZone = -1;
  return navGoTo(x, y, F("patient slot"));
}

// ---- zones ----
const __FlashStringHelper *zoneName(Zone z) {
  switch (z) {
    case ZONE_HOSPITAL: return F("HOSPITAL");
    case ZONE_PCC1:     return F("PCC1");
    case ZONE_PCC2:     return F("PCC2");
    default:            return F("RECOVERY");
  }
}

// RED -> Hospital, YELLOW -> PCC of that side, GREEN -> Recovery
Zone zoneFor(PatientColor c, bool rightSide) {
  if (c == COL_RED)    return ZONE_HOSPITAL;
  if (c == COL_YELLOW) return rightSide ? ZONE_PCC2 : ZONE_PCC1;
  return ZONE_RECOVERY;
}

static void zoneNominal(Zone z, float &x, float &y) {
  switch (z) {
    case ZONE_HOSPITAL: x = HOSP_X;  y = HOSP_Y;  break;
    case ZONE_PCC1:     x = PCC1_X;  y = PCC1_Y;  break;
    case ZONE_PCC2:     x = PCC2_X;  y = PCC2_Y;  break;
    default:            x = RECOV_X; y = RECOV_Y; break;
  }
  if (st.zone[z].ok) { x += st.zone[z].dx; y += st.zone[z].dy; }
}

// dropIndex 0,1,2,3,4 -> 0, +s, -s, +2s, -2s  (patients do not land on each other)
static float dropBack = 0;          // v18h: set only by navToDrop

// v18h: patient drop = zone point, but DROP_BACK_..._MM shorter in X (towards H CENTER)
bool navToDrop(Zone z, byte dropIndex) {
  switch (z) {
    case ZONE_HOSPITAL: dropBack = DROP_BACK_HOSP_MM;  break;
    case ZONE_PCC1:     dropBack = DROP_BACK_PCC1_MM;  break;
    case ZONE_PCC2:     dropBack = DROP_BACK_PCC2_MM;  break;
    default:            dropBack = DROP_BACK_RECOV_MM; break;
  }
  if (dropBack != 0) {
    Serial.print(F("DROP: stopping "));
    Serial.print(dropBack, 0);
    Serial.println(F(" mm short of the zone"));
  }
  bool ok = navToZone(z, dropIndex);
  dropBack = 0;
  return ok;
}

bool navToZone(Zone z, byte dropIndex) {
  float x, y;
  zoneNominal(z, x, y);
  if (dropBack != 0) x += (x > 0 ? -dropBack : dropBack);   // shorter = towards H CENTER
  if (dropIndex > 0) {
    float k = (float)((dropIndex + 1) / 2);
    y += ((dropIndex % 2) ? 1.0 : -1.0) * k * DROP_SPREAD_MM;
  }
  if (!navGoTo(x, y, zoneName(z))) return false;
  navLastZone = z;                 // so the way back can use the return assist
  return true;
}