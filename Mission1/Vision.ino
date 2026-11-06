// ======================================================
// Vision.ino  -  GROVE VISION AI V2 (I2C) + lookForSample
// ======================================================
#include "Config.h"

#define GV2_ADDR        0x62
#define GV2_FEATURE     0x10
#define GV2_CMD_READ    0x01
#define GV2_CMD_WRITE   0x02
#define GV2_CMD_AVAIL   0x03
#define GV2_CHUNK       24     // AVR Wire buffer = 32 bytes
#define GV2_LINE_MAX    600
#define GV2_TIMEOUT_MS  1500

Box boxes[MAX_BOXES];
int boxCount = 0;
bool visionOk = false;

char gvLine[GV2_LINE_MAX];
int  gvLen = 0;

bool gv2Present()
{
  Wire.beginTransmission(GV2_ADDR);
  return Wire.endTransmission() == 0;
}

void gv2Write(const char *data, uint16_t len)
{
  uint16_t sent = 0;
  while (sent < len)
  {
    uint16_t n = len - sent;
    if (n > GV2_CHUNK) n = GV2_CHUNK;

    Wire.beginTransmission(GV2_ADDR);
    Wire.write(GV2_FEATURE);
    Wire.write(GV2_CMD_WRITE);
    Wire.write((uint8_t)(n >> 8));
    Wire.write((uint8_t)(n & 0xFF));
    Wire.write((const uint8_t *)data + sent, n);
    Wire.write((uint8_t)0);   // checksum (unused)
    Wire.write((uint8_t)0);
    Wire.endTransmission();

    sent += n;
  }
}

uint16_t gv2Available()
{
  Wire.beginTransmission(GV2_ADDR);
  Wire.write(GV2_FEATURE);
  Wire.write(GV2_CMD_AVAIL);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  if (Wire.endTransmission() != 0)
    return 0;

  delay(2);
  if (Wire.requestFrom((uint8_t)GV2_ADDR, (uint8_t)2) != 2)
    return 0;

  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return (hi << 8) | lo;
}

uint8_t gv2Read(uint8_t *buf, uint8_t len)   // len <= 32
{
  Wire.beginTransmission(GV2_ADDR);
  Wire.write(GV2_FEATURE);
  Wire.write(GV2_CMD_READ);
  Wire.write((uint8_t)0);
  Wire.write(len);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  if (Wire.endTransmission() != 0)
    return 0;

  delay(2);
  uint8_t got = Wire.requestFrom((uint8_t)GV2_ADDR, len);
  for (uint8_t i = 0; i < got; i++)
    buf[i] = Wire.read();
  return got;
}

// Throw away any old data waiting in the Grove
void gv2Flush()
{
  uint8_t tmp[32];
  unsigned long t0 = millis();
  uint16_t n;
  while ((n = gv2Available()) > 0 && millis() - t0 < 300)
    gv2Read(tmp, n > 32 ? 32 : (uint8_t)n);
}

// Parse  "boxes": [[x,y,w,h,score,target], ...]
void gv2ParseBoxes(const char *p)
{
  boxCount = 0;
  p = strchr(p, '[');
  if (!p) return;
  p++;

  while (*p && boxCount < MAX_BOXES)
  {
    while (*p == ' ' || *p == ',') p++;
    if (*p != '[') break;
    p++;

    double v[6];
    int k = 0;
    while (k < 6)
    {
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

    if (k == 6)
    {
      Box &b = boxes[boxCount++];
      b.x = (int)v[0];
      b.y = (int)v[1];
      b.w = (int)v[2];
      b.h = (int)v[3];
      // score may come as 0-100 or 0.0-1.0
      b.score = (v[4] <= 1.0) ? (int)(v[4] * 100.0 + 0.5) : (int)v[4];
      b.target = (int)v[5];
    }
  }
}

// 1 = boxes received, 0 = keep waiting, -1 = error reply
int gv2HandleLine()
{
  if (!strstr(gvLine, "INVOKE"))
    return 0;

  const char *c = strstr(gvLine, "\"code\"");
  if (c)
  {
    c = strchr(c, ':');
    if (c && atoi(c + 1) != 0)
      return -1;
  }

  const char *b = strstr(gvLine, "\"boxes\"");
  if (!b)
    return 0;   // first ACK reply, result comes next

  gv2ParseBoxes(b);
  return 1;
}

// Run one inference. Returns 0 on success.
int gv2Invoke()
{
  gv2Flush();

  const char cmd[] = "AT+INVOKE=1,0,1\r\n";   // 1 frame, results only
  gv2Write(cmd, strlen(cmd));

  bool inLine = false;
  gvLen = 0;
  uint8_t buf[32];
  unsigned long t0 = millis();

  while (millis() - t0 < GV2_TIMEOUT_MS)
  {
    uint16_t n = gv2Available();
    if (n == 0)
    {
      delay(5);
      continue;
    }

    while (n > 0)
    {
      uint8_t want = n > 32 ? 32 : (uint8_t)n;
      uint8_t got = gv2Read(buf, want);
      if (got == 0) break;
      n = (got >= n) ? 0 : n - got;

      for (uint8_t i = 0; i < got; i++)
      {
        char ch = (char)buf[i];

        if (!inLine)
        {
          if (ch == '{')
          {
            inLine = true;
            gvLen = 0;
            gvLine[gvLen++] = ch;
          }
          continue;
        }

        if (ch == '\n')
        {
          gvLine[gvLen] = 0;
          inLine = false;
          int r = gv2HandleLine();
          if (r == 1)  return 0;
          if (r == -1) return -1;
          continue;
        }

        if (gvLen < GV2_LINE_MAX - 1)
          gvLine[gvLen++] = ch;
        else
          inLine = false;   // too long, drop
      }
    }
  }

  return -2;   // timeout
}

// Look for the sample for up to timeoutMs.
// On success x, y = sample position in the picture (average of the frames).
bool lookForSample(int &x, int &y, unsigned long timeoutMs) {
  if (!visionOk) {
    visionOk = gv2Present();
  }

  if (!visionOk) {
    Serial.println(F("Camera: Grove NOT FOUND on I2C (0x62)"));
    return false;
  }

  int hits = 0;
  long sumX = 0, sumY = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (abortRequested()) return false;

    if (gv2Invoke() != 0) {
      hits = 0; sumX = 0; sumY = 0;
      continue;
    }

    int best = -1;
    for (int i = 0; i < boxCount; i++) {
      if (boxes[i].target == SAMPLE_TARGET_ID && boxes[i].score >= MIN_SCORE) {
        if (best < 0 || boxes[i].score > boxes[best].score) best = i;
      }
    }

    if (best < 0) {
      hits = 0; sumX = 0; sumY = 0;
      continue;
    }

    hits++;
    sumX += boxes[best].x;
    sumY += boxes[best].y;

    if (hits >= CONFIRM_FRAMES) {
      x = (int)(sumX / hits);
      y = (int)(sumY / hits);

      Serial.print(F("Camera: SAMPLE seen  score="));
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

// Is the sample STILL near (tx,ty)?  Used to check that a pick worked:
// after the arm lifted, a sample that is still on the floor is seen again
// at the same place. Returns true (and its position) if seen VERIFY_FRAMES
// frames in a row within 'radius' pixels.
bool sampleNear(int tx, int ty, int radius, unsigned long timeoutMs, int &ox, int &oy) {
  if (!visionOk) {
    visionOk = gv2Present();
  }
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
      if (boxes[i].target == SAMPLE_TARGET_ID && boxes[i].score >= MIN_SCORE) {
        long dx = boxes[i].x - tx;
        long dy = boxes[i].y - ty;
        long d2 = dx * dx + dy * dy;
        if (d2 <= r2 && d2 < bestD) {
          bestD = d2;
          best = i;
        }
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

// 9: CAMERA TEST. For 5 seconds every picture of the camera is shown, so you can see
// whether the camera answers, what it sees and why a sample is not accepted.
void cameraTest() {
  Serial.println();
  Serial.println(F("CAMERA TEST (5 s): put ONE sample in front of the arm (inside the camera view)."));
  Serial.print(F("The code accepts: class number "));
  Serial.print(SAMPLE_TARGET_ID);
  Serial.print(F("  with a score of at least "));
  Serial.println(MIN_SCORE);

  visionOk = gv2Present();
  if (!visionOk) {
    Serial.println(F("RESULT: the camera is NOT on the I2C bus (0x62). Check its power and the SDA / SCL wires."));
    return;
  }

  int frames = 0, answered = 0, withBoxes = 0, accepted = 0;
  int otherClass = -1;
  int bestOtherScore = 0;
  int lowScore = 0;
  unsigned long t0 = millis();

  while (millis() - t0 < 5000) {
    if (abortRequested()) return;

    int r = gv2Invoke();
    frames++;

    if (r != 0) {
      Serial.print(F("picture "));
      Serial.print(frames);
      Serial.print(F(": NO ANSWER from the camera (code "));
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
      Serial.print(F("  score "));
      Serial.print(boxes[i].score);
      Serial.print(F("  x="));
      Serial.print(boxes[i].x);
      Serial.print(F(" y="));
      Serial.println(boxes[i].y);

      if (boxes[i].target == SAMPLE_TARGET_ID && boxes[i].score >= MIN_SCORE) accepted++;
      else if (boxes[i].target == SAMPLE_TARGET_ID) lowScore = boxes[i].score;
      else if (boxes[i].score > bestOtherScore) { bestOtherScore = boxes[i].score; otherClass = boxes[i].target; }
    }
  }

  Serial.println();
  Serial.print(F("SUMMARY: "));
  Serial.print(frames);
  Serial.print(F(" pictures, answered "));
  Serial.print(answered);
  Serial.print(F(", with something seen "));
  Serial.print(withBoxes);
  Serial.print(F(", ACCEPTED as a sample "));
  Serial.println(accepted);

  if (answered == 0) {
    Serial.println(F("RESULT: the camera does NOT answer. Close the PC tool / SenseCraft page that is connected to the camera by USB, then power-cycle the camera and the Mega."));
  } else if (accepted > 0) {
    Serial.println(F("RESULT: the camera sees the sample and it is accepted. The camera is fine."));
    Serial.println(F("        If K still says NOT FOUND, the sample was outside the view at the sweep stops."));
  } else if (withBoxes == 0) {
    Serial.println(F("RESULT: the camera answers but sees NOTHING. Is the sample in the view and well lit? Is the lens covered?"));
  } else if (otherClass >= 0) {
    Serial.print(F("RESULT: the camera sees something with class number "));
    Serial.print(otherClass);
    Serial.print(F(" (score "));
    Serial.print(bestOtherScore);
    Serial.println(F("), but the code only accepts SAMPLE_TARGET_ID. If that object is the sample, set SAMPLE_TARGET_ID in Config.h to that number."));
  } else {
    Serial.print(F("RESULT: the sample is seen but its score ("));
    Serial.print(lowScore);
    Serial.println(F(") is below MIN_SCORE. Better light, or lower MIN_SCORE in Config.h."));
  }
}
