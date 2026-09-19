# Hardware Connection

## Devices in this system

| Device | Role |
|---|---|
| OV5647 5MP CSI camera | Mounted on Grove Vision AI V2 (supersedes the earlier Sunny P5V04A in this project's diagram — same CSI role) |
| Seeed Studio Grove Vision AI V2 | Runs on-device AI inference (Ethos-U55 NPU) |
| XIAO (plugged into GV2's onboard expansion header) | Relays GV2's detection results to the Mega over UART, using Seeed's `Seeed_Arduino_SSCMA` library — see `arduino/xiao_gv2_bridge/xiao_gv2_bridge.ino` |
| Arduino Mega 2560 | Final robot controller, receives detections, drives arm/motors |
| PC (this development machine) | Debug / calibration / visualization ONLY — not part of the competition-time data path |

**Updated final data path (no PC in the loop at competition time):**
```
OV5647 -> Grove Vision AI V2 -> XIAO (UART, onboard header) -> Arduino Mega -> arm
```
The PC-side Python scripts (`gv2_serial_test.py`, `gv2_detect.py`) still work
exactly as before for debugging over USB — GV2 supports both the XIAO-header
UART and the USB-C AT-command link at the same time; they are separate
consumers of the same on-device inference.

## Camera -> GV2

The Sunny P5V04A connects to GV2 via its CSI connector. Confirm orientation of the
FPC ribbon cable matches Seeed's documented orientation (contacts facing the
correct direction per [Grove Vision AI V2 hardware docs](https://wiki.seeedstudio.com/grove_vision_ai_v2/)) —
a reversed or half-seated ribbon is a common cause of "camera not detected"
that looks like a software problem but isn't. **NOT HARDWARE VERIFIED here** —
I cannot see your physical cable seating from this environment.

Camera checks to run on-device (see `docs/DEPLOYMENT.md` step 3-4 for the
commands that exercise these):
- image arrives at all (a live GV2 preview shows something)
- correct color (not swapped R/B, not monochrome if you expect color)
- reasonable brightness/exposure for your competition lighting
- stable frame-to-frame (no flicker, no dropped frames)
- latency: measure with `gv2_detect.py`'s per-invoke timing print, not by eye

**Do not substitute your laptop webcam for any of this.** A webcam is a
different sensor with different color science, FOV, and no CSI/Ethos-U path
at all — pipeline behavior with it tells you nothing about the deployed
system.

## GV2 -> PC (development/debug link only)

- Physical: USB-C
- Windows port: COM13 (yours may differ — check Device Manager)
- Baud: 921600 (confirmed working in this project's earlier testing)
- This link is used by `python/gv2_serial_test.py`, `python/gv2_detect.py`,
  and `python/gv2_to_mega_bridge.py`. It is **not** part of the final
  competition data path.

## GV2 -> XIAO (onboard header, no external wiring)

GV2 has a built-in XIAO expansion connector; the UART between GV2 and a
seated XIAO board is carried by that connector, not loose wires.
`arduino/xiao_gv2_bridge/xiao_gv2_bridge.ino` opens it as
`HardwareSerial atSerial(0)` and talks to it with Seeed's
`Seeed_Arduino_SSCMA` library (real API, not guessed — see that file's
header comment for the working reference it was based on).

## XIAO -> Arduino Mega 2560 (final competition data path)

- Physical: a second UART, wired externally — **common GND required**
- Mega side: `Serial1` — RX1 = pin 19, TX1 = pin 18
- XIAO side: `MEGA_TX_PIN` / `MEGA_RX_PIN` constants at the top of
  `xiao_gv2_bridge.ino` (defaulted to GPIO2/GPIO1 for XIAO ESP32-S3 as a
  starting point) — **confirm these are free/correct on your exact XIAO
  board's pinout before wiring; not hardware-verified from here.**
- `Serial` (USB) on both the Mega and the XIAO is reserved for debug prints
  to your laptop, separate from this link.
- **Do not use SoftwareSerial** on the Mega side — it has four real hardware UARTs; use one of them.

The line protocol on this link
(`D,<class_id>,<confidence_x100>,<x>,<y>,<w>,<h>,<cx_x10>,<cy_x10>\n`) is
identical whether it comes from `xiao_gv2_bridge.ino` (competition setup) or
`python/gv2_to_mega_bridge.py` (PC-based development/testing stand-in) —
`gv2_to_mega.ino` on the Mega needs no changes either way. See
`docs/SERIAL_PROTOCOL.md`.

## Arduino Mega -> Robot / Arm

Not specified in the uploaded materials (no motor driver, servo count, or arm
kinematics were provided). `arduino/gv2_to_mega/gv2_to_mega.ino` stops at
printing the parsed, mapped detection and marks exactly where your
motor/servo calls should go (`routeDetection()`), rather than guessing your
actuator wiring.

## Power / grounding

Whatever your GV2, camera, and Mega power sources are, tie all grounds
together. This is the single most common cause of garbled/absent UART data
between boards on separate supplies and cannot be verified from here —
check it physically.
