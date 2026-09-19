#!/usr/bin/env python3
"""
gv2_serial_test.py
===================
Minimal, one-shot diagnostic tool for the Grove Vision AI V2 over serial.
Run this FIRST, before gv2_detect.py, whenever something seems wrong.

It does exactly one thing at a time and prints the raw device response, so
you can see precisely what the firmware is telling you rather than trusting
a downstream script's interpretation.

Protocol reference (verified against Seeed-Studio/SSCMA-Micro docs/protocol/
at_protocol.md, 1.0.x branch -- not guessed):
  - Command framing:  "<COMMAND>\r"
  - Response framing: "\r<json>\n"
  - AT+MODEL?           -> {"type":0,"name":"MODEL?","code":0,
                             "data":{"id":..,"type":..,"address":..,"size":..}}
                          size == 0 means NO valid model is actually resident
                          in that model slot right now (this is exactly what
                          was observed earlier on this project and is a
                          strong signal to re-flash before touching anything
                          else).
  - AT+INFO?            -> {"type":0,"name":"INFO?","code":0,
                             "data":{"crc16_maxim":..,"info":"..."}}
  - AT+INVOKE=1,0,1     -> triggers ONE inference. Produces TWO JSON lines:
                             1) type 0 "operation" reply (echoes model/algorithm/sensor)
                             2) type 1 "event" reply with the actual result:
                                {"type":1,"name":"INVOKE","code":0,
                                 "data":{"count":N,"perf":[...],
                                         "boxes":[[x,y,w,h,score,target_id], ...]}}
                          boxes == [] with code == 0 means: inference ran,
                          but nothing cleared the on-device score threshold /
                          decode step. That is a genuinely different failure
                          from "no model loaded" (size == 0 above) or a
                          camera problem -- see docs/TROUBLESHOOTING.md.

Usage:
    python gv2_serial_test.py --port COM13 --baud 921600 --cmd model
    python gv2_serial_test.py --port COM13 --baud 921600 --cmd info
    python gv2_serial_test.py --port COM13 --baud 921600 --cmd invoke
    python gv2_serial_test.py --port COM13 --baud 921600 --cmd raw --raw-command "AT+ID?"
"""
import argparse
import json
import sys
import time

import serial

# =============================================================================
# VS CODE QUICK-RUN CONFIG -- edit COM_PORT, then just press Run (▷).
# =============================================================================
COM_PORT = "COM13"
BAUD_RATE = 921600
DEFAULT_CMD = "invoke"  # "model" | "info" | "invoke" | "raw"
# =============================================================================


def send_command(ser: serial.Serial, command: str, timeout_s: float = 3.0, max_lines: int = 2):
    """Send one AT command and read back up to `max_lines` framed JSON responses.

    Framing per SSCMA-Micro at_protocol.md: request is "<CMD>\\r"; each
    response is "\\r<json>\\n". We read raw bytes until we've collected
    `max_lines` newline-terminated frames or timeout elapses -- NOT inside
    any connection callback (see docs/TROUBLESHOOTING.md item on the
    recursive-Invoke bug this project hit before).
    """
    ser.reset_input_buffer()
    ser.write((command + "\r").encode("ascii"))
    ser.flush()

    frames = []
    buf = b""
    deadline = time.time() + timeout_s
    while time.time() < deadline and len(frames) < max_lines:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("utf-8", errors="replace").strip("\r").strip()
            if not text:
                continue
            frames.append(text)
    return frames


def parse_frames(frames):
    parsed = []
    for f in frames:
        try:
            parsed.append(json.loads(f))
        except json.JSONDecodeError:
            print(f"  [WARN] non-JSON line from device (printed raw): {f!r}")
    return parsed


def cmd_model(ser):
    frames = send_command(ser, "AT+MODEL?", max_lines=1)
    for f in frames:
        print("RAW:", f)
    for obj in parse_frames(frames):
        data = obj.get("data", {})
        size = data.get("size")
        print(json.dumps(obj, indent=2))
        if size == 0:
            print("\n*** size == 0: NO valid model is currently resident on GV2. ***")
            print("*** Re-flash best_full_integer_quant_vela.tflite via SenseCraft ***")
            print("*** (or the official flashing tool) before doing anything else. ***")
        else:
            print(f"\nModel present: id={data.get('id')} type={data.get('type')} "
                  f"address={data.get('address')} size={size} bytes")


def cmd_info(ser):
    frames = send_command(ser, "AT+INFO?", max_lines=1)
    for f in frames:
        print("RAW:", f)
    for obj in parse_frames(frames):
        print(json.dumps(obj, indent=2))


def cmd_invoke(ser):
    # Safe sequence per project requirement: connect -> initialize -> wait ->
    # invoke -> receive -> parse. This function assumes connect+wait already
    # happened (serial.Serial(...) succeeded and we gave the board a moment).
    frames = send_command(ser, "AT+INVOKE=1,0,1", max_lines=2, timeout_s=5.0)
    for f in frames:
        print("RAW:", f)
    objs = parse_frames(frames)
    for obj in objs:
        print(json.dumps(obj, indent=2))
    event = next((o for o in objs if o.get("type") == 1), None)
    if event is None:
        print("\n*** No type=1 event reply received within timeout. Either the model ***")
        print("*** isn't loaded (run --cmd model first) or inference didn't complete. ***")
        return
    boxes = event.get("data", {}).get("boxes", [])
    perf = event.get("data", {}).get("perf")
    print(f"\nperf (ms): {perf}")
    if not boxes:
        print("*** boxes == [] : inference ran but produced ZERO detections. ***")
        print("*** This is the exact failure mode this project needs to fix. ***")
        print("*** See docs/TROUBLESHOOTING.md for the ranked list of causes. ***")
    else:
        names = ["red_patient", "yellow_patient", "green_patient", "sample"]
        for b in boxes:
            x, y, w, h, score, target = b
            cls = names[target] if 0 <= target < len(names) else f"class_{target}"
            print(f"  {cls:14s} score={score:>4} x={x} y={y} w={w} h={h}")


def cmd_raw(ser, raw_command):
    frames = send_command(ser, raw_command, max_lines=2)
    for f in frames:
        print("RAW:", f)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=COM_PORT, help="e.g. COM13")
    ap.add_argument("--baud", type=int, default=BAUD_RATE)
    ap.add_argument("--cmd", choices=["model", "info", "invoke", "raw"], default=DEFAULT_CMD)
    ap.add_argument("--raw-command", default=None)
    args = ap.parse_args()

    print(f"Opening {args.port} @ {args.baud} ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        print(f"FAILED to open {args.port}: {e}")
        sys.exit(1)

    time.sleep(0.3)  # let the board settle -- do NOT invoke from inside any connect callback

    if args.cmd == "model":
        cmd_model(ser)
    elif args.cmd == "info":
        cmd_info(ser)
    elif args.cmd == "invoke":
        cmd_invoke(ser)
    elif args.cmd == "raw":
        if not args.raw_command:
            ap.error("--cmd raw requires --raw-command")
        cmd_raw(ser, args.raw_command)

    ser.close()
