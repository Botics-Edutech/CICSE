#!/usr/bin/env python3
"""
gv2_detect.py
=============
Final PC-side companion program for Grove Vision AI V2.

What this does:
  - Opens COM13 @ 921600 baud (both overridable via CLI flags).
  - Sends AT+INVOKE with SSCMA-Micro's real framing (see gv2_serial_test.py
    for the protocol reference -- verified against Seeed-Studio/SSCMA-Micro
    docs/protocol/at_protocol.md, not guessed).
  - Parses the REAL response shape: {"type":1,"data":{"count":N,"perf":[...],
    "boxes":[[x,y,w,h,score,target_id], ...]}}.
  - Maps target_id -> class name using the fixed 0..3 mapping required by
    this project (red_patient, yellow_patient, green_patient, sample).
  - Prints every detection to the terminal.
  - If the response also carries an "image" field (base64 JPEG -- present
    when RESULT_ONLY=0), decodes and displays it with OpenCV, draws boxes,
    class, confidence, and X/Y/W/H, and computes the box center (cx, cy).
  - Supports multiple simultaneous detections -- does not stop at the first.
  - Runs GV2's ON-DEVICE inference only. Never loads best.pt, never runs
    YOLO on the PC. This file is PC-side visualization/debugging/logging
    only, per this project's architecture requirement.

Safety / architecture notes (do not change without re-reading why):
  - Invoke is NEVER called from a serial "on_connect" callback. The flow is
    strictly: open port -> short settle delay -> loop { invoke -> read -> parse }.
    An earlier version of this project's SSCMA callback code invoked
    recursively from inside on_connect, which is why this version uses a
    plain, single-threaded read loop instead of an event-driven callback API.

Usage:
    python gv2_detect.py --port COM13 --baud 921600 --threshold 0.5
    python gv2_detect.py --port COM13 --once            # single-shot, no GUI loop
"""
import argparse
import base64
import json
import sys
import time
from dataclasses import dataclass, asdict
from typing import Optional

import serial

# =============================================================================
# VS CODE QUICK-RUN CONFIG
# If you just press the "Run" (▷) button in VS Code with no arguments, THESE
# defaults are what get used. Edit COM_PORT to match your Device Manager entry
# (e.g. "COM13" on Windows, "/dev/ttyACM0" or "/dev/ttyUSB0" on Linux/Mac)
# and you can run this file directly, no terminal flags needed.
# =============================================================================
COM_PORT = "COM13"
BAUD_RATE = 921600
CONFIDENCE_THRESHOLD = 0.5
# =============================================================================

# The GV2 model (models_v2_yolov8n) takes a 192x192 input, so every box in
# the AT+INVOKE response ("boxes":[[x,y,w,h,score,target_id],...]) is
# reported in that 192x192 coordinate space -- NOT in the resolution of the
# preview JPEG the device streams back in the "image" field. Those two are
# different sizes (the preview is the device's native/cropped frame, often
# larger than 192x192), which is exactly why boxes drawn with raw
# coordinates land offset from the object: same top-left origin, wrong
# scale. Fixing this requires rescaling box coordinates into whatever the
# decoded preview frame's actual pixel size turns out to be, computed fresh
# from frame.shape every time (never hardcoded), before drawing OR printing.
MODEL_INPUT_SIZE = 192

# Real-hardware test (frame confirmed square, 240x240, so scale alone is
# exact) still showed the box consistently shifted right+down from the
# object by roughly half the box's own width/height. That specific pattern
# means SSCMA is reporting (x,y) as the box CENTER (the standard YOLO
# cx,cy,w,h convention), not the top-left corner this code assumed. Set to
# False and re-test if this turns out to be wrong for your firmware version.
BOX_XY_IS_CENTER = True

CLASS_NAMES = ["red_patient", "yellow_patient", "green_patient", "sample"]

CLASS_COLOR_BGR = {
    "red_patient": (0, 0, 220),
    "yellow_patient": (0, 220, 220),
    "green_patient": (0, 160, 0),
    "sample": (30, 30, 30),
}


@dataclass
class Detection:
    class_id: int
    class_name: str
    confidence: float
    x: int
    y: int
    w: int
    h: int
    center_x: float
    center_y: float


def class_name(target_id: int) -> str:
    if 0 <= target_id < len(CLASS_NAMES):
        return CLASS_NAMES[target_id]
    return f"class_{target_id}"


def read_frame(ser: serial.Serial, timeout_s: float) -> Optional[str]:
    """Read one '\\r<json>\\n' framed response. Returns the JSON text or None."""
    buf = b""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        if b"\n" in buf:
            line, _, _rest = buf.partition(b"\n")
            return line.decode("utf-8", errors="replace").strip("\r").strip()
    return None


def invoke_once(ser: serial.Serial, result_only: bool, timeout_s: float = 5.0):
    """Send one AT+INVOKE and return (operation_obj, event_obj_or_None)."""
    ser.reset_input_buffer()
    flag = 1 if result_only else 0
    ser.write(f"AT+INVOKE=1,0,{flag}\r".encode("ascii"))
    ser.flush()

    op_obj, event_obj = None, None
    deadline = time.time() + timeout_s
    while time.time() < deadline and event_obj is None:
        text = read_frame(ser, max(0.05, deadline - time.time()))
        if not text:
            continue
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            continue
        if obj.get("name") != "INVOKE":
            continue
        if obj.get("type") == 0:
            op_obj = obj
        elif obj.get("type") == 1:
            event_obj = obj
    return op_obj, event_obj


def to_detections(event_obj: dict, threshold: float, scale_x: float = 1.0, scale_y: float = 1.0):
    """scale_x/scale_y convert from the model's 192x192 box coordinate space
    into whatever pixel space the caller wants the returned Detection
    objects expressed in (normally the actual decoded preview frame's
    width/height, computed by the caller as frame_w/MODEL_INPUT_SIZE and
    frame_h/MODEL_INPUT_SIZE). Leave both at 1.0 only if you deliberately
    want raw 192x192-space coordinates (e.g. no image was returned, as in
    --result-only mode, so there is no frame to scale against)."""
    detections = []
    if not event_obj:
        return detections
    boxes = event_obj.get("data", {}).get("boxes", [])
    for b in boxes:
        if len(b) < 6:
            continue
        x, y, w, h, score, target = b[:6]
        conf = float(score) / 100.0 if score > 1 else float(score)  # SSCMA scores are 0-100 ints
        if conf < threshold:
            continue
        x = x * scale_x
        y = y * scale_y
        w = w * scale_x
        h = h * scale_y

        if BOX_XY_IS_CENTER:
            cx, cy = x, y
            x_topleft = cx - w / 2
            y_topleft = cy - h / 2
        else:
            x_topleft, y_topleft = x, y
            cx = x_topleft + w / 2
            cy = y_topleft + h / 2

        detections.append(Detection(
            class_id=int(target), class_name=class_name(int(target)),
            confidence=conf, x=int(round(x_topleft)), y=int(round(y_topleft)),
            w=int(round(w)), h=int(round(h)),
            center_x=cx, center_y=cy,
        ))
    return detections


def print_detections(detections):
    if not detections:
        print("  (no detections above threshold)")
        return
    for d in detections:
        print("Detection:")
        print(f"  Class: {d.class_name}  (target_id: {d.class_id})")
        print(f"  Confidence: {d.confidence:.2f}")
        print(f"  X: {d.x}  Y: {d.y}  W: {d.w}  H: {d.h}")
        print(f"  CX: {d.center_x:.1f}  CY: {d.center_y:.1f}")


def draw_detections(frame, detections):
    """Draws, per detection: bounding box, class+confidence label above the
    box, and an X/Y/W/H/target_id line below the box (matches the on-screen
    layout requested: each object gets its own labeled box with its raw
    x,y,w,h,confidence,target_id visible). Also draws a small top-left
    summary panel ("GV2 CAMERA | Objects: N") like a HUD header."""
    import cv2

    # --- HUD header panel (top-left) ---
    header = f"GV2 CAMERA | Objects: {len(detections)}"
    (tw, th), _ = cv2.getTextSize(header, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    cv2.rectangle(frame, (5, 5), (15 + tw, 15 + th), (0, 0, 0), -1)
    cv2.putText(frame, header, (10, 10 + th), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    for d in detections:
        color = CLASS_COLOR_BGR.get(d.class_name, (255, 255, 255))
        x1, y1, x2, y2 = d.x, d.y, d.x + d.w, d.y + d.h

        # bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # class + confidence label above the box, on a filled tag background
        label = f"{d.class_name} {d.confidence:.2f}"
        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        ly = max(0, y1 - 8)
        cv2.rectangle(frame, (x1, ly - lh - 6), (x1 + lw + 6, ly + 4), color, -1)
        cv2.putText(frame, label, (x1 + 3, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # raw x/y/w/h + target_id line below the box
        info = f"X:{d.x} Y:{d.y} W:{d.w} H:{d.h} id:{d.class_id}"
        cv2.putText(frame, info, (x1, y2 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        # center point marker
        cv2.circle(frame, (int(d.center_x), int(d.center_y)), 3, color, -1)

    return frame


def decode_image_field(event_obj: dict):
    data = event_obj.get("data", {}) if event_obj else {}
    img_b64 = data.get("image")
    if not img_b64:
        return None
    try:
        import numpy as np
        import cv2
        raw = base64.b64decode(img_b64)
        arr = np.frombuffer(raw, dtype="uint8")
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    except Exception as e:
        print(f"  [WARN] could not decode image field: {e}")
        return None


def debug_frame_shape(frame):
    """TEMPORARY diagnostic: prints the real decoded preview frame size so we
    can confirm whether it's square (plain uniform scale is enough) or not
    (device is letterboxing/cropping to feed the 192x192 model, and the
    box-to-pixel mapping needs a padding-aware inverse transform, not just a
    scale). Remove this call once the mapping is confirmed correct."""
    if frame is not None:
        print(f"  [DEBUG] actual frame shape: width={frame.shape[1]} height={frame.shape[0]}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=COM_PORT)
    ap.add_argument("--baud", type=int, default=BAUD_RATE)
    ap.add_argument("--threshold", type=float, default=CONFIDENCE_THRESHOLD,
                     help="confidence threshold (0-1); tune with 0.3/0.4/0.5/0.6 on real hardware")
    ap.add_argument("--result-only", action="store_true", default=False,
                     help="skip image transfer for lower latency (no OpenCV preview)")
    ap.add_argument("--once", action="store_true", help="run a single invoke and exit")
    ap.add_argument("--interval", type=float, default=0.0,
                     help="seconds to sleep between invokes in continuous mode")
    args = ap.parse_args()

    print(f"Opening {args.port} @ {args.baud} ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        print(f"FAILED to open {args.port}: {e}")
        sys.exit(1)
    time.sleep(0.3)  # settle -- invoke is issued from the loop below, never from a connect callback
    print("GV2 CONNECTED")

    have_cv2 = False
    if not args.result_only:
        try:
            import cv2  # noqa: F401
            have_cv2 = True
        except ImportError:
            print("OpenCV not installed -- running without image preview (pip install opencv-python)")

    try:
        while True:
            t0 = time.time()
            op_obj, event_obj = invoke_once(ser, result_only=args.result_only)
            t_total = (time.time() - t0) * 1000
            if event_obj is None:
                print(f"[{t_total:.0f} ms] no result from GV2 (check `python gv2_serial_test.py --cmd model`)")
            else:
                perf = event_obj.get("data", {}).get("perf")
                print(f"[{t_total:.0f} ms total | device perf(ms)={perf}]")

                # Decode the preview frame FIRST (if present) so we know its
                # real pixel size, then scale the model's 192x192-space boxes
                # into that frame's coordinate space. This is the fix for
                # boxes appearing offset from the object: the frame is not
                # 192x192, so raw box coordinates were being drawn/printed at
                # the wrong scale.
                frame = decode_image_field(event_obj) if have_cv2 else None
                debug_frame_shape(frame)
                if frame is not None:
                    scale_x = frame.shape[1] / MODEL_INPUT_SIZE
                    scale_y = frame.shape[0] / MODEL_INPUT_SIZE
                else:
                    scale_x = scale_y = 1.0  # no frame to scale against (e.g. --result-only)

                detections = to_detections(event_obj, args.threshold, scale_x, scale_y)
                print_detections(detections)

                if frame is not None:
                    import cv2
                    frame = draw_detections(frame, detections)
                    cv2.imshow("GV2 Detection", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

            if args.once:
                break
            if args.interval > 0:
                time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if have_cv2:
            import cv2
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()