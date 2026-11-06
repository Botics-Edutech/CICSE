// ======================================================
// Vision.ino  -  GROVE VISION AI V2 (I2C)  (driver same as Mission 1)
// Mission 2: a "sample" = any PATIENT (red / green / yellow).
// lookForSample() also sets lastColor (majority of the frames).
// ======================================================
#include "Config.h"

#define GV2_ADDR        0x62
#define GV2_FEATURE     0x10
#define GV2_CMD_READ    0x01
#define GV2_CMD_WRITE   0x02
#define GV2_CMD_AVAIL   0x03
#define GV2_CHUNK       24
#define GV2_LINE_MAX    600
#define GV2_TIMEOUT_MS  1500

Box boxes[MAX_BOXES];
int boxCount = 0;
bool visionOk = false;
PatientColor lastColor = COL_NONE;
int lastBoxW = 0, lastBoxH = 0;     // size of the last patient seen (px)

char gvLine[GV2_LINE_MAX];
int  gvLen = 0;

bool gv2Present() {
  Wire.beginTransmission(GV2_ADDR);
  return Wire.endTransmission() == 0;
}

static void gv2Write(const char *data, uint16_t len) {
  uint16_t sent = 0;
  while (sent < len) {
    uint16_t n = len - sent;
    if (n > GV2_CHUNK) n = GV2_CHUNK;

    Wire.beginTransmission(GV2_ADDR);
    Wire.write(GV2_FEATURE);
    Wire.write(GV2_CMD_WRITE);
    Wire.write((uint8_t)(n >> 8));
    Wire.write((uint8_t)(n & 0xFF));
    Wire.write((const uint8_t *)data + sent, n);
    Wire.write((uint8_t)0);
    Wire.write((uint8_t)0);
    Wire.endTransmission();
    sent += n;
  }
}

static uint16_t gv2Available() {
  Wire.beginTransmission(GV2_ADDR);
  Wire.write(GV2_FEATURE);
  Wire.write(GV2_CMD_AVAIL);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  if (Wire.endTransmission() != 0) return 0;

  delay(2);
  if (Wire.requestFrom((uint8_t)GV2_ADDR, (uint8_t)2) != 2) return 0;

  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return (hi << 8) | lo;
}

static uint8_t gv2Read(uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(GV2_ADDR);
  Wire.write(GV2_FEATURE);
  Wire.write(GV2_CMD_READ);
  Wire.write((uint8_t)0);
  Wire.write(len);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  if (Wire.endTransmission() != 0) return 0;

  delay(2);
  uint8_t got = Wire.requestFrom((uint8_t)GV2_ADDR, len);
  for (uint8_t i = 0; i < got; i++) buf[i] = Wire.read();
  return got;
}

static void gv2Flush() {
  uint8_t tmp[32];
  unsigned long t0 = millis();
  uint16_t n;
  while ((n = gv2Available()) > 0 && millis() - t0 < 300)
    gv2Read(tmp, n > 32 ? 32 : (uint8_t)n);
}

static void gv2ParseBoxes(const char *p) {
  boxCount = 0;
  p = strchr(p, '[');
  if (!p) return;
  p++;

  while (*p && boxCount < MAX_BOXES) {
    while (*p == ' ' || *p == ',') p++;
    if (*p != '[') break;
    p++;

    double v[6];
    int k = 0;
    while (k < 6) {
      while (*p == ' ' || *p == ',') p++;
      if (*p == ']' || *p == 0) break;
      char *e;
      v[k] = strtod(p, &e);
      if (e == p) return;
      p = e;
      k++;
    }

    while (*p && *p != ']') p++;
    if (*p == ']') p++;

    if (k == 6) {
      Box &b = boxes[boxCount++];
      b.x = (int)v[0];
      b.y = (int)v[1];
      b.w = (int)v[2];
      b.h = (int)v[3];
      b.score = (v[4] <= 1.0) ? (int)(v[4] * 100.0 + 0.5) : (int)v[4];
      b.target = (int)v[5];
#if !BOX_XY_IS_CENTER
      // camera sends the top-left corner: move x,y to the CENTRE of the box
      b.x = (int)(v[0] + v[2] / 2.0 + 0.5);
      b.y = (int)(v[1] + v[3] / 2.0 + 0.5);
#endif
      // from here on b.x, b.y is ALWAYS the centre of the patient
    }
  }
}

static int gv2HandleLine() {
  if (!strstr(gvLine, "INVOKE")) return 0;

  const char *c = strstr(gvLine, "\"code\"");
  if (c) {
    c = strchr(c, ':');
    if (c && atoi(c + 1) != 0) return -1;
  }

  const char *b = strstr(gvLine, "\"boxes\"");
  if (!b) return 0;

  gv2ParseBoxes(b);
  return 1;
}

// One inference. 0 = OK, -1 = error reply, -2 = timeout
int gv2Invoke() {
  gv2Flush();

  const char cmd[] = "AT+INVOKE=1,0,1\r\n";
  gv2Write(cmd, strlen(cmd));

  bool inLine = false;
  gvLen = 0;
  uint8_t buf[32];
  unsigned long t0 = millis();

  while (millis() - t0 < GV2_TIMEOUT_MS) {
#if USE_MPU
    mpuUpdate();
#endif
    uint16_t n = gv2Available();
    if (n == 0) {
      delay(5);
      continue;
    }

    while (n > 0) {
      uint8_t want = n > 32 ? 32 : (uint8_t)n;
      uint8_t got = gv2Read(buf, want);
      if (got == 0) break;
      n = (got >= n) ? 0 : n - got;

      for (uint8_t i = 0; i < got; i++) {
        char ch = (char)buf[i];

        if (!inLine) {
          if (ch == '{') {
            inLine = true;
            gvLen = 0;
            gvLine[gvLen++] = ch;
          }
          continue;
        }

        if (ch == '\n') {
          gvLine[gvLen] = 0;
          inLine = false;
          int r = gv2HandleLine();
          if (r == 1)  return 0;
          if (r == -1) return -1;
          continue;
        }

        if (gvLen < GV2_LINE_MAX - 1) gvLine[gvLen++] = ch;
        else inLine = false;
      }
    }
  }
  return -2;
}

// ------------------------------------------------------
// PATIENT CLASSES
// ------------------------------------------------------
PatientColor classToColor(int t) {
  if (t == CLASS_RED)    return COL_RED;
  if (t == CLASS_GREEN)  return COL_GREEN;
  if (t == CLASS_YELLOW) return COL_YELLOW;
  return COL_NONE;
}

bool isPatientClass(int t) { return classToColor(t) != COL_NONE; }

const char *colorName(PatientColor c) {
  switch (c) {
    case COL_RED:    return "RED";
    case COL_GREEN:  return "GREEN";
    case COL_YELLOW: return "YELLOW";
    default:         return "NONE";
  }
}

// Where the patient should be in the picture: the taught target (G), else the middle.
int g_aimX = IMAGE_CENTER_X, g_aimY = IMAGE_CENTER_Y;

// The patient nearest to this pixel is the one we want (neighbours are ignored)
void setAim(int x, int y) { g_aimX = x; g_aimY = y; }

static void aimPoint(int &ax, int &ay) { ax = g_aimX; ay = g_aimY; }

// Box touches the picture edge -> the patient is partly outside the picture,
// so the box is too small on that side and its centre is wrong.
bool boxCut(const Box &b) {
  int l = b.x - b.w / 2, r = b.x + b.w / 2;
  int t = b.y - b.h / 2, d = b.y + b.h / 2;
  return l < EDGE_MARGIN_PX || t < EDGE_MARGIN_PX ||
         r > IMAGE_W - EDGE_MARGIN_PX || d > IMAGE_H - EDGE_MARGIN_PX;
}

// The patient box nearest the aim point (neighbours 100 mm away are ignored)
// skipCut = ignore boxes cut off by the picture edge (for aiming)
static int bestPatientBox(bool skipCut) {
  int ax, ay;
  aimPoint(ax, ay);
  int best = -1;
  long bestD = 0x7FFFFFFFL;
  for (int i = 0; i < boxCount; i++) {
    if (!isPatientClass(boxes[i].target) || boxes[i].score < MIN_SCORE) continue;
    if (skipCut && boxCut(boxes[i])) continue;
    long dx = boxes[i].x - ax, dy = boxes[i].y - ay;
    long d2 = dx * dx + dy * dy;
    if (d2 < bestD) { bestD = d2; best = i; }
  }
  return best;
}

// Look for a patient for up to timeoutMs. On success x, y = its position
// (average of CONFIRM_FRAMES frames) and lastColor = its colour.
bool lookForSample(int &x, int &y, unsigned long timeoutMs) {
  if (!visionOk) visionOk = gv2Present();
  if (!visionOk) {
    Serial.println(F("Camera: Grove NOT FOUND on I2C (0x62)"));
    return false;
  }

  int hits = 0;
  long sumX = 0, sumY = 0;
  byte votes[4] = {0, 0, 0, 0};
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (abortRequested()) return false;

    if (gv2Invoke() != 0) {
      hits = 0; sumX = 0; sumY = 0; votes[1] = votes[2] = votes[3] = 0;
      continue;
    }

    int best = bestPatientBox(false);
    if (best < 0) {
      hits = 0; sumX = 0; sumY = 0; votes[1] = votes[2] = votes[3] = 0;
      continue;
    }

    hits++;
    sumX += boxes[best].x;
    sumY += boxes[best].y;
    votes[classToColor(boxes[best].target)]++;

    if (hits >= CONFIRM_FRAMES) {
      x = (int)(sumX / hits);
      y = (int)(sumY / hits);

      PatientColor c = COL_RED;
      if (votes[COL_GREEN] > votes[c]) c = COL_GREEN;
      if (votes[COL_YELLOW] > votes[c]) c = COL_YELLOW;
      lastColor = c;
      lastBoxW = boxes[best].w;
      lastBoxH = boxes[best].h;

      Serial.print(F("Camera: PATIENT "));
      Serial.print(colorName(c));
      Serial.print(F("  score="));
      Serial.print(boxes[best].score);
      Serial.print(F("  x="));
      Serial.print(x);
      Serial.print(F("  y="));
      Serial.println(y);
      return true;
    }
  }
  return false;
}

static int medianOf(int *a, byte n) {
  for (byte i = 1; i < n; i++) {            // small insertion sort
    int v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
  return (n & 1) ? a[n / 2] : (a[n / 2 - 1] + a[n / 2]) / 2;
}

// PRECISE look for aiming (fine correction + teach).
// Uses the box centre (x, y with w, h), ignores boxes cut by the picture edge,
// and takes the MIDDLE value of ALIGN_FRAMES pictures (one bad picture cannot pull it away).
// x, y = centre of the patient   w, h = its size in pixels
bool lookPrecise(int &x, int &y, int &w, int &h, unsigned long timeoutMs) {
  if (!visionOk) visionOk = gv2Present();
  if (!visionOk) return false;

  int xs[ALIGN_FRAMES], ys[ALIGN_FRAMES], ws[ALIGN_FRAMES], hs[ALIGN_FRAMES];
  byte n = 0;
  bool cutSeen = false;
  unsigned long start = millis();

  while (n < ALIGN_FRAMES && millis() - start < timeoutMs) {
    if (abortRequested()) return false;
    if (gv2Invoke() != 0) continue;
    int best = bestPatientBox(true);
    if (best < 0) {
      if (bestPatientBox(false) >= 0) cutSeen = true;
      continue;
    }
    xs[n] = boxes[best].x;
    ys[n] = boxes[best].y;
    ws[n] = boxes[best].w;
    hs[n] = boxes[best].h;
    n++;
  }

  if (n < ALIGN_MIN_FRAMES) {
    if (cutSeen) Serial.println(F("   precise look: patient CUT by the picture edge - not used"));
    return false;
  }
  x = medianOf(xs, n);
  y = medianOf(ys, n);
  w = medianOf(ws, n);
  h = medianOf(hs, n);
  lastBoxW = w;
  lastBoxH = h;
  Serial.print(F("   precise: centre x="));
  Serial.print(x);
  Serial.print(F(" y="));
  Serial.print(y);
  Serial.print(F("  size "));
  Serial.print(w);
  Serial.print('x');
  Serial.print(h);
  Serial.print(F("  ("));
  Serial.print(n);
  Serial.println(F(" pictures)"));
  return true;
}

// PICK CHECK with proof.
//   1 = picked     (camera answered VERIFY_CLEAR_FRAMES times, no patient near (tx,ty) in ANY)
//   0 = NOT picked (a patient was seen near (tx,ty) - ox, oy = where)
//  -1 = not sure   (camera did not answer often enough) -> caller treats it as NOT picked
int pickCheck(int tx, int ty, int radius, int &ox, int &oy) {
  if (!visionOk) visionOk = gv2Present();
  if (!visionOk) {
    Serial.println(F("CHECK: camera NOT found"));
    return -1;
  }
  long r2 = (long)radius * (long)radius;
  byte answered = 0;
  unsigned long start = millis();
  // size of the patient ON THE FLOOR (from the aiming look). A patient held in the
  // claw is much closer to the camera -> its box is much BIGGER -> it is not "on the floor".
#if GRIPPER_CLAW && CLAW_HELD_VISIBLE
  int floorSize = lastBoxW > lastBoxH ? lastBoxW : lastBoxH;
#endif

  // v14: the first pictures after an arm move can be OLD (taken before the move) -> throw away
  for (byte k = 0; k < VERIFY_SKIP_FRAMES && millis() - start < VERIFY_MS; ) {
    if (abortRequested()) return -1;
    if (gv2Invoke() == 0) k++;
  }
  start = millis();

  while (millis() - start < VERIFY_MS) {
    if (abortRequested()) return -1;
    if (gv2Invoke() != 0) continue;          // no answer: does NOT count as "gone"
    answered++;

    for (int i = 0; i < boxCount; i++) {
      if (!isPatientClass(boxes[i].target) || boxes[i].score < VERIFY_MIN_SCORE) continue;
#if GRIPPER_CLAW && CLAW_HELD_VISIBLE
      int sz = boxes[i].w > boxes[i].h ? boxes[i].w : boxes[i].h;
      if (floorSize > 5 && sz > (int)(VERIFY_HELD_SIZE * floorSize)) {
        Serial.print(F("CHECK: big patient (size "));
        Serial.print(sz);
        Serial.print(F(" vs floor "));
        Serial.print(floorSize);
        Serial.println(F(") = held in the CLAW - ignored"));
        continue;
      }
#endif
      long dx = boxes[i].x - tx, dy = boxes[i].y - ty;
      if (dx * dx + dy * dy <= r2) {
        ox = boxes[i].x;
        oy = boxes[i].y;
        Serial.print(F("CHECK: patient STILL there (picture "));
        Serial.print(answered);
        Serial.print(F(", score "));
        Serial.print(boxes[i].score);
        Serial.print(F(", moved "));
        Serial.print((int)sqrt((float)(dx * dx + dy * dy)));
        Serial.println(F(" px)"));
        return 0;
      }
    }
    if (answered >= VERIFY_CLEAR_FRAMES) {
      Serial.print(F("CHECK: "));
      Serial.print(answered);
      Serial.print(F(" pictures, nothing within "));
      Serial.print(radius);
      Serial.println(F(" px"));
      return 1;
    }
  }
  Serial.print(F("CHECK: camera answered only "));
  Serial.print(answered);
  Serial.println(F(" times - NOT SURE"));
  return -1;
}

// Is a patient STILL near (tx,ty)?  (old check, not used any more)
bool sampleNear(int tx, int ty, int radius, unsigned long timeoutMs, int &ox, int &oy) {
  if (!visionOk) visionOk = gv2Present();
  if (!visionOk) return false;

  int hits = 0;
  long sumX = 0, sumY = 0;
  unsigned long start = millis();
  long r2 = (long)radius * (long)radius;

  while (millis() - start < timeoutMs) {
    if (abortRequested()) return false;

    if (gv2Invoke() != 0) {
      hits = 0; sumX = 0; sumY = 0;
      continue;
    }

    int best = -1;
    long bestD = 0x7FFFFFFFL;
    for (int i = 0; i < boxCount; i++) {
      if (isPatientClass(boxes[i].target) && boxes[i].score >= MIN_SCORE) {
        long dx = boxes[i].x - tx;
        long dy = boxes[i].y - ty;
        long d2 = dx * dx + dy * dy;
        if (d2 <= r2 && d2 < bestD) { bestD = d2; best = i; }
      }
    }

    if (best < 0) {
      hits = 0; sumX = 0; sumY = 0;
      continue;
    }

    hits++;
    sumX += boxes[best].x;
    sumY += boxes[best].y;

    if (hits >= VERIFY_FRAMES) {
      ox = (int)(sumX / hits);
      oy = (int)(sumY / hits);
      return true;
    }
  }
  return false;
}

// 9: CAMERA TEST (5 s). Shows every class number the camera sees.
// Put ONE patient (red, then green, then yellow) under the camera
// and write down its class number -> CLASS_RED / GREEN / YELLOW in Config.h
void cameraTest() {
  Serial.println();
  Serial.println(F("CAMERA TEST (5 s): put ONE patient under the camera."));
  Serial.print(F("Config: RED="));
  Serial.print(CLASS_RED);
  Serial.print(F(" GREEN="));
  Serial.print(CLASS_GREEN);
  Serial.print(F(" YELLOW="));
  Serial.print(CLASS_YELLOW);
  Serial.print(F("  MIN_SCORE="));
  Serial.println(MIN_SCORE);

  visionOk = gv2Present();
  if (!visionOk) {
    Serial.println(F("RESULT: camera NOT on the I2C bus (0x62). Check power and SDA / SCL."));
    return;
  }

  int frames = 0, answered = 0, withBoxes = 0;
  int classCount[8] = {0};
  unsigned long t0 = millis();

  while (millis() - t0 < 5000) {
    if (abortRequested()) return;

    int r = gv2Invoke();
    frames++;

    if (r != 0) {
      Serial.print(F("picture "));
      Serial.print(frames);
      Serial.print(F(": NO ANSWER (code "));
      Serial.print(r);
      Serial.println(F(")"));
      continue;
    }

    answered++;
    if (boxCount > 0) withBoxes++;

    Serial.print(F("picture "));
    Serial.print(frames);
    Serial.print(F(": boxes="));
    Serial.println(boxCount);

    for (int i = 0; i < boxCount; i++) {
      Serial.print(F("     class "));
      Serial.print(boxes[i].target);
      Serial.print(F(" ("));
      Serial.print(colorName(classToColor(boxes[i].target)));
      Serial.print(F(")  score "));
      Serial.print(boxes[i].score);
      Serial.print(F("  x="));
      Serial.print(boxes[i].x);
      Serial.print(F(" y="));
      Serial.print(boxes[i].y);
      Serial.print(F(" w="));
      Serial.print(boxes[i].w);
      Serial.print(F(" h="));
      Serial.print(boxes[i].h);
      Serial.println(boxCut(boxes[i]) ? F("  CUT by edge") : F(""));
      if (boxes[i].target >= 0 && boxes[i].target < 8) classCount[boxes[i].target]++;
    }
  }

  Serial.println();
  Serial.print(F("SUMMARY: "));
  Serial.print(frames);
  Serial.print(F(" pictures, answered "));
  Serial.print(answered);
  Serial.print(F(", with something seen "));
  Serial.println(withBoxes);
  for (int k = 0; k < 8; k++) {
    if (!classCount[k]) continue;
    Serial.print(F("   class "));
    Serial.print(k);
    Serial.print(F(" seen "));
    Serial.print(classCount[k]);
    Serial.print(F(" times  -> code calls it "));
    Serial.println(colorName(classToColor(k)));
  }

  if (answered == 0)
    Serial.println(F("RESULT: camera does NOT answer. Close SenseCraft / USB tool, power-cycle camera + Mega."));
  else if (withBoxes == 0)
    Serial.println(F("RESULT: camera answers but sees NOTHING. Patient in view? Light OK?"));
  else
    Serial.println(F("RESULT: camera works. If the colour name is wrong, fix CLASS_* in Config.h."));
}