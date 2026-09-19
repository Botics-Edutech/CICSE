#!/usr/bin/env python3
"""
gv2_to_mega_bridge.py
======================
Development-only PC-side bridge: GV2 (COM13, SSCMA-Micro AT protocol) -> this
script -> Arduino Mega (a second serial port, plain line protocol).

Architecture reminder from this project's spec (section 19):
    "If the official GV2 protocol can be directly parsed by Mega, implement
    that. If not, clearly explain the limitation and create the simplest
    robust serial protocol possible."
Raw SSCMA JSON parsing on an 8-bit AVR Mega is impractical (no reliable
heap-based JSON parsing, limited RAM). This bridge exists SOLELY so you can
test the Mega firmware and robot-control logic today. The end-competition
target architecture is still GV2 -> UART -> Mega directly; this script is a
development stand-in for that link until/unless GV2 firmware exposes a
simplified passthrough Mega could read natively.

Line protocol sent to the Mega (see docs/SERIAL_PROTOCOL.md and
arduino/gv2_to_mega/gv2_to_mega.ino):
    D,<class_id>,<confidence_x100>,<x>,<y>,<w>,<h>,<cx_x10>,<cy_x10>\n

Usage:
    python gv2_to_mega_bridge.py --gv2-port COM13 --gv2-baud 921600 \
                                  --mega-port COM7 --mega-baud 115200 \
                                  --threshold 0.5
"""
import argparse
import json
import sys
import time

import serial

CLASS_NAMES = ["red_patient", "yellow_patient", "green_patient", "sample"]


def read_frame(ser, timeout_s):
    buf = b""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        if b"\n" in buf:
            line, _, _ = buf.partition(b"\n")
            return line.decode("utf-8", errors="replace").strip("\r").strip()
    return None


def invoke_once(gv2, timeout_s=5.0):
    gv2.reset_input_buffer()
    gv2.write(b"AT+INVOKE=1,0,1\r")
    gv2.flush()
    deadline = time.time() + timeout_s
    event_obj = None
    while time.time() < deadline and event_obj is None:
        text = read_frame(gv2, max(0.05, deadline - time.time()))
        if not text:
            continue
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            continue
        if obj.get("name") == "INVOKE" and obj.get("type") == 1:
            event_obj = obj
    return event_obj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gv2-port", required=True)
    ap.add_argument("--gv2-baud", type=int, default=921600)
    ap.add_argument("--mega-port", required=True)
    ap.add_argument("--mega-baud", type=int, default=115200)
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--interval", type=float, default=0.0)
    args = ap.parse_args()

    gv2 = serial.Serial(args.gv2_port, args.gv2_baud, timeout=0.2)
    time.sleep(0.3)
    mega = serial.Serial(args.mega_port, args.mega_baud, timeout=0.2)
    time.sleep(2.0)  # Mega resets on serial open; give bootloader time to finish
    print(f"Bridge running: GV2({args.gv2_port}) -> Mega({args.mega_port})")

    try:
        while True:
            event = invoke_once(gv2)
            if event is None:
                time.sleep(args.interval if args.interval > 0 else 0.05)
                continue
            for b in event.get("data", {}).get("boxes", []):
                if len(b) < 6:
                    continue
                x, y, w, h, score, target = b[:6]
                conf = score / 100.0 if score > 1 else score
                if conf < args.threshold:
                    continue
                cx = x + w / 2
                cy = y + h / 2
                line = f"D,{int(target)},{int(round(conf * 100))},{int(x)},{int(y)},{int(w)},{int(h)},{int(round(cx * 10))},{int(round(cy * 10))}\n"
                mega.write(line.encode("ascii"))
                name = CLASS_NAMES[target] if 0 <= target < len(CLASS_NAMES) else f"class_{target}"
                print(f"-> Mega: {name} conf={conf:.2f} cx={cx:.1f} cy={cy:.1f}")
            if args.interval > 0:
                time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        gv2.close()
        mega.close()


if __name__ == "__main__":
    main()
