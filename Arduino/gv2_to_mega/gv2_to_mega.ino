/*
 * gv2_to_mega.ino
 * ================
 * Arduino Mega 2560 receiver for Grove Vision AI V2 detection results.
 *
 * IMPORTANT / HONESTY NOTE:
 * The exact SSCMA-Micro AT+INVOKE JSON framing IS documented (verified
 * against Seeed-Studio/SSCMA-Micro docs/protocol/at_protocol.md):
 *   request:  "AT+INVOKE=1,0,1\r"
 *   response: "\r{...json...}\n"   (an operation reply, then an event reply)
 * Parsing full JSON on an 8-bit AVR Mega with no heap-friendly JSON library
 * pre-installed is possible but fragile. Per this project's own instruction
 * ("if raw SSCMA output is difficult to parse directly on Mega, create a
 * PC-side bridge first for testing"), THIS FIRMWARE DOES NOT PARSE RAW GV2
 * JSON ITSELF. Instead it expects a SIMPLE, FIXED, comma-separated line from
 * whatever is upstream on Serial1 -- either:
 *   (a) gv2_to_mega_bridge.py (see docs/SERIAL_PROTOCOL.md) running on a PC
 *       between GV2 and the Mega during development, or
 *   (b) a future firmware-side JSON parser on GV2/ESP-side hardware that
 *       reformats to this same line format, if the official SSCMA firmware
 *       ever exposes a raw UART passthrough for Mega to read directly.
 *
 * This keeps AI parsing and robot-control logic separated, as required, and
 * gives you a WORKING Mega receiver today rather than a half-finished raw
 * JSON parser. If Seeed's firmware turns out to support direct one-line
 * detection output on a secondary UART, swap out only parseLine() below.
 *
 * ---------------------------------------------------------------------
 * LINE PROTOCOL (see docs/SERIAL_PROTOCOL.md for the full spec):
 *   D,<class_id>,<confidence_x100>,<x>,<y>,<w>,<h>,<cx_x10>,<cy_x10>\n
 * Example:
 *   D,0,94,87,83,77,65,1255,1155\n
 *     -> class_id=0 (red_patient), confidence=0.94, x=87,y=83,w=77,h=65,
 *        cx=125.5, cy=115.5
 * A line with no detections in a cycle is simply not sent (Mega just sees
 * nothing new that loop iteration).
 * ---------------------------------------------------------------------
 *
 * Wiring:
 *   GV2/PC-bridge TX -> Mega RX1 (pin 19)
 *   GV2/PC-bridge RX <- Mega TX1 (pin 18)   (only needed if you send ACKs)
 *   Common GND between all boards -- REQUIRED.
 *   USB (Serial, pins via USB connector) used for debug prints to your PC.
 *
 * Do NOT use SoftwareSerial: the Mega has four real hardware UARTs
 * (Serial, Serial1, Serial2, Serial3) -- Serial1 is used here as required.
 */

#define LINK Serial1          // GV2 / PC-bridge link
#define LINK_BAUD 115200      // baud between bridge and Mega (independent of GV2's 921600 to PC)
#define DEBUG_BAUD 115200

const char *CLASS_NAMES[4] = {"red_patient", "yellow_patient", "green_patient", "sample"};

// Class -> destination mapping (AI only detects/reports; Mega decides action).
const char *DESTINATION[4] = {"Hospital", "PCC", "Recovery Zone", "Laboratory"};

struct Detection {
  int classId;
  float confidence;
  int x, y, w, h;
  float cx, cy;
  bool valid;
};

String lineBuf;

void setup() {
  Serial.begin(DEBUG_BAUD);
  LINK.begin(LINK_BAUD);
  while (!Serial) { ; }
  Serial.println(F("Mega ready. Waiting for GV2 detections on Serial1..."));
  lineBuf.reserve(96);
}

void loop() {
  while (LINK.available()) {
    char c = LINK.read();
    if (c == '\n') {
      handleLine(lineBuf);
      lineBuf = "";
    } else if (c != '\r') {
      lineBuf += c;
      if (lineBuf.length() > 120) {
        lineBuf = "";  // guard against garbage/overflow
      }
    }
  }
}

void handleLine(const String &line) {
  if (line.length() == 0) return;

  Detection d;
  if (!parseLine(line, d)) {
    Serial.print(F("Unparsed line: "));
    Serial.println(line);
    return;
  }

  Serial.println(F("GV2 DETECTION"));
  Serial.print(F("Class: "));
  Serial.println(className(d.classId));
  Serial.print(F("Confidence: "));
  Serial.println(d.confidence, 2);
  Serial.print(F("CX: "));
  Serial.println(d.cx, 1);
  Serial.print(F("CY: "));
  Serial.println(d.cy, 1);

  routeDetection(d);
}

const char *className(int id) {
  if (id >= 0 && id < 4) return CLASS_NAMES[id];
  return "unknown";
}

// Parses: D,<class_id>,<confidence_x100>,<x>,<y>,<w>,<h>,<cx_x10>,<cy_x10>
bool parseLine(const String &line, Detection &out) {
  if (line.charAt(0) != 'D' || line.charAt(1) != ',') return false;

  int fields[8];
  int fieldCount = 0;
  int start = 2;
  for (int i = 2; i <= line.length() && fieldCount < 8; i++) {
    if (i == line.length() || line.charAt(i) == ',') {
      String tok = line.substring(start, i);
      fields[fieldCount++] = tok.toInt();
      start = i + 1;
    }
  }
  if (fieldCount != 8) return false;

  out.classId = fields[0];
  out.confidence = fields[1] / 100.0f;
  out.x = fields[2];
  out.y = fields[3];
  out.w = fields[4];
  out.h = fields[5];
  out.cx = fields[6] / 10.0f;
  out.cy = fields[7] / 10.0f;
  out.valid = true;
  return true;
}

// ---- Robot-control decision logic lives HERE, separated from AI parsing ----
void routeDetection(const Detection &d) {
  if (d.classId < 0 || d.classId > 3) return;

  // NOTE: this only prints the intended destination. Actual motor/arm
  // commands (coordinate mapping -> arm target -> pick/place sequence) are
  // deliberately NOT wired up here: this project's calibration.py has to
  // produce a validated homography from YOUR real robot geometry first (see
  // docs/DEPLOYMENT.md step 6). Wire your servo/stepper driver calls in the
  // block below once that calibration exists -- do not hard-code coordinates.
  Serial.print(F("-> Route to: "));
  Serial.println(DESTINATION[d.classId]);

  // Example of where robot_x/robot_y (from the PC-side calibration mapping,
  // sent over as part of a richer line protocol if you extend it) would feed
  // an arm-target function:
  //   moveArmTo(robot_x, robot_y);
  //   pick();
  //   place(DESTINATION[d.classId]);
}
