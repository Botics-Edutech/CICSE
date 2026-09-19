/*
 * xiao_gv2_bridge.ino
 * ====================
 * Matches this updated architecture:
 *
 *   OV5647 5MP CSI camera -> Grove Vision AI V2 (on-device inference)
 *       -> class+confidence, x/y/w/h -> XIAO (this sketch, UART/I2C)
 *       -> Arduino Mega 2560 -> coordinate mapping -> robotic arm -> pick/place
 *
 * This runs on the XIAO board that plugs into Grove Vision AI V2's onboard
 * XIAO expansion connector. It uses Seeed's own `Seeed_Arduino_SSCMA`
 * Arduino library to talk to GV2 (NOT the raw AT-command JSON parsing used
 * by the PC-side Python scripts) -- this is the correct, native way to do
 * it from a microcontroller, and it is the same underlying protocol
 * (verified from a working Seeed community XIAO ESP32-S3 + GV2 example
 * using this exact library -- API calls below are copied from real,
 * working code, not invented):
 *
 *     SSCMA AI;
 *     AI.begin(&atSerial);
 *     AI.invoke(1, false, true);           // n_times=1, differed=false, result_only=true
 *     AI.boxes()[i].x / .y / .w / .h / .score / .target
 *
 * `AI.invoke(1, false, true)` is the library's wrapper around the same
 * `AT+INVOKE=1,0,1` command documented in docs/SERIAL_PROTOCOL.md -- the
 * numbers mean the same thing.
 *
 * WHAT THIS SKETCH DOES NOT DO:
 *   - It does not do CSI camera setup itself -- GV2 handles the OV5647
 *     directly; XIAO only talks to GV2's UART command interface.
 *   - It does not run any AI on the XIAO -- GV2's Ethos-U55 does all
 *     inference; XIAO just relays results.
 *   - It does not compute robot coordinates -- that stays on the Mega
 *     (coordinate mapping / calibration.py's homography), keeping AI and
 *     robot-control logic separated per this project's requirement.
 *
 * HARDWARE / WIRING ASSUMPTIONS -- confirm before flashing:
 *   - GV2 <-> XIAO: no wiring needed. GV2's onboard XIAO expansion header
 *     carries this UART directly when the XIAO is seated in it. This
 *     sketch opens that link as `HardwareSerial atSerial(0)` (UART0),
 *     matching the working reference example.
 *   - XIAO <-> Arduino Mega: THIS requires real wiring, since it is a
 *     second, separate UART on whichever free GPIO pins your XIAO variant
 *     exposes. The pin numbers below (MEGA_TX_PIN / MEGA_RX_PIN) are a
 *     starting point for XIAO ESP32-S3 -- verify against your exact board's
 *     pinout diagram before wiring; change them if those pins are already
 *     used by something else on your XIAO. Baud between XIAO and Mega is
 *     independent of the GV2 link and set to 115200 here.
 *   - Tie GND common between XIAO, GV2, and Mega.
 *
 * NOT HARDWARE VERIFIED: I do not have physical access to your XIAO/GV2/
 * Mega. This sketch is written against the library's real, documented API
 * and a working community reference, but compiling and flashing it is
 * something only you can do and confirm.
 *
 * Line protocol sent to Mega (matches arduino/gv2_to_mega/gv2_to_mega.ino
 * and docs/SERIAL_PROTOCOL.md exactly -- that Mega sketch needs NO changes,
 * it already reads this same format on its Serial1 regardless of whether
 * it came from this XIAO or the PC bridge script):
 *
 *   D,<class_id>,<confidence_x100>,<x>,<y>,<w>,<h>,<cx_x10>,<cy_x10>\n
 */

#include <Seeed_Arduino_SSCMA.h>
#ifdef ESP32
#include <HardwareSerial.h>
#endif

// ---- CONFIG: verify against your exact XIAO board before flashing ----
#define MEGA_TX_PIN 2   // XIAO GPIO used to TRANSMIT to Mega's RX1 (pin 19)
#define MEGA_RX_PIN 1   // XIAO GPIO used to RECEIVE from Mega's TX1 (pin 18) -- optional, only if Mega talks back
#define MEGA_BAUD 115200
#define CONFIDENCE_THRESHOLD 50   // 0-100 scale, matches SSCMA's integer score; tune from real testing (30/40/50/60)
// ------------------------------------------------------------------------

HardwareSerial atSerial(0);      // UART0: GV2, via the XIAO expansion header (no external wiring)
HardwareSerial megaSerial(1);    // UART1: to Arduino Mega (wire per MEGA_TX_PIN / MEGA_RX_PIN above)

SSCMA AI;

const char *CLASS_NAMES[4] = {"red_patient", "yellow_patient", "green_patient", "sample"};

void setup() {
  Serial.begin(921600);          // USB debug console to your PC (optional, for `Serial Monitor`)
  delay(200);

  AI.begin(&atSerial);           // connect to GV2 over the header UART
  Serial.println("GV2 (via XIAO) initialized");

  megaSerial.begin(MEGA_BAUD, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);
  Serial.println("Mega UART ready");

  // NOTE: invoke is never called here in setup()/on-connect -- it only
  // happens in loop(), avoiding the recursive-Invoke bug this project hit
  // before with a different (Python/callback-based) SSCMA client.
}

void loop() {
  if (!AI.invoke(1, false, true)) {   // n_times=1, differed=false, result_only=true == AT+INVOKE=1,0,1
    int n = AI.boxes().size();
    if (n == 0) {
      Serial.println("invoke ok, boxes=[] (no detections this frame)");
    }
    for (int i = 0; i < n; i++) {
      auto &box = AI.boxes()[i];
      if (box.score < CONFIDENCE_THRESHOLD) continue;

      int classId = box.target;
      const char *name = (classId >= 0 && classId < 4) ? CLASS_NAMES[classId] : "unknown";
      float cx = box.x + box.w / 2.0f;
      float cy = box.y + box.h / 2.0f;

      Serial.print("Detection: ");
      Serial.print(name);
      Serial.print(" score="); Serial.print(box.score);
      Serial.print(" x="); Serial.print(box.x);
      Serial.print(" y="); Serial.print(box.y);
      Serial.print(" w="); Serial.print(box.w);
      Serial.print(" h="); Serial.println(box.h);

      // Forward to Mega using the same line protocol the PC bridge uses --
      // gv2_to_mega.ino does not need to know or care which one sent it.
      char line[80];
      snprintf(line, sizeof(line), "D,%d,%d,%d,%d,%d,%d,%d,%d\n",
               classId, box.score, box.x, box.y, box.w, box.h,
               (int)(cx * 10), (int)(cy * 10));
      megaSerial.print(line);
    }
  } else {
    Serial.println("AI.invoke() failed -- check GV2 connection / model load "
                    "(cross-check with `python gv2_serial_test.py --cmd model`)");
  }

  delay(20);  // small pacing gap; tune once real inference latency is measured
}
