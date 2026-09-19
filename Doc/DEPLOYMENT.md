# Deployment Guide

## 0. What you already have, verified

`models/best_full_integer_quant_vela.tflite` (== `models/BOTICS_ALPHA_GV2_FINAL.tflite`,
identical bytes, two filenames for convenience) has been structurally validated:

```
python tools/validate_tflite.py models/best_full_integer_quant_vela.tflite
```

- Input: `[1,224,224,3]` INT8, NHWC, scale 0.00392, zero_point -128 -- correct full-integer quant.
- Output: `[1,8,1029]` INT8 -- 4 bbox coords + 4 classes, 1029 anchors (28²+14²+7² at strides 8/16/32 for 224 input). This is the **standard, un-NMS'd YOLO26 raw detection head** -- confirmed by re-exporting `best.pt` with the current Ultralytics toolchain and observing the same `[1,8,1029]` shape (see step 2 below for why we did NOT keep that fresh export).
- Compiled to a single `ethos-u` custom op with **zero CPU-fallback ops** -- 100% of the graph is offloaded to the Ethos-U55 NPU. This is the cleanest possible Vela compile outcome.

**Conclusion: this file is structurally correct and is the one to flash.** Do not regenerate it from `best.pt` using this environment's toolchain (see next section for why).

## 1. Why we did NOT re-export from best.pt

We attempted the "obvious" fix path (re-export `best.pt` -> int8 tflite -> Vela) to see if it would improve anything. It did not — it reproduces exactly the failure pattern described in this project's own history:

```
python tools/validate_tflite.py models/best_full_integer_quant_vela.tflite \
    --compare-fresh-export --pt models/best.pt --calib-yaml <a data.yaml with images>
```

The current `ultralytics==8.4.89` + its `litert_torch` TFLite export backend produces a model with:
- **NCHW** input layout instead of NHWC,
- a **float32 boundary** wrapped around the int8 core (an explicit `QUANTIZE` op on input, `DEQUANTIZE` op on output),
- and when Vela-compiles that, it emits exactly: `Warning (supported operators) operator: DEQUANTIZE ... Unsupported opType` and `Warning ... operator: QUANTIZE ... unsupported DataType Float32` — **this is the same warning class described in your project notes about the previous bad Vela pipeline.**

So: the tool available in this environment today would make things *worse*, not better. Your already-provided file is better than what current tooling reproduces, which strongly suggests it was built by a different (older/onnx2tf-based) export path — the one Seeed's official Colab notebook actually uses. **Keep it.**

## 2. Flashing to Grove Vision AI V2

1. Put GV2 in flashing mode via SenseCraft AI (https://sensecraft.seeed.cc) or the official Model Assistant flashing tool, following [Seeed's Grove Vision AI V2 deployment guide](https://wiki.seeedstudio.com/grove_vision_ai_v2_sscma/).
2. Upload `models/best_full_integer_quant_vela.tflite` as a **custom model**, along with the class labels in this exact order (do not let the tool auto-generate different names):
   ```
   0 red_patient
   1 yellow_patient
   2 green_patient
   3 sample
   ```
3. After flashing, **immediately verify the flash actually took**, before assuming anything about detection quality:
   ```
   python python/gv2_serial_test.py --port COM13 --baud 921600 --cmd model
   ```
   You are looking for `"size"` to be a real non-zero byte count (roughly matching the 2.4 MB file). **`"size": 0` means the flash did not take and nothing below will work** — this exact symptom was seen earlier in this project. Re-flash before doing anything else if you see it.

## 3. First inference test

```
python python/gv2_serial_test.py --port COM13 --baud 921600 --cmd invoke
```

Point the camera at a single, well-lit red/yellow/green/sample object first (not all four at once) to reduce variables. You are looking for a non-empty `boxes` array. If you get `boxes: []` here even with a confirmed-loaded model, work through `docs/TROUBLESHOOTING.md` in order — do not guess.

## 4. Continuous detection with visualization

```
python python/gv2_detect.py --port COM13 --baud 921600 --threshold 0.5
```

Test thresholds 0.3 / 0.4 / 0.5 / 0.6 against the real scene, per the project spec — pick the practical value from what you actually observe, not from theory.

## 5. Full scene test (all four objects)

Reproduce the reference photo: red_patient, yellow_patient, green_patient, sample all in frame simultaneously. Confirm `gv2_detect.py` reports 4 simultaneous detections with the correct class IDs and reasonable confidence. **This is the actual bar for success (spec section 28: A+B+C+D+E+F+G+H+I) — until this step passes on the real board, nothing above it should be called "working."**

## 6. Robot integration

1. Run `python python/calibration.py collect` then `fit` to build `homography.json` from real measured points (see the module docstring — do not skip this by hard-coding coordinates).
2. Bridge GV2 -> Mega for development: `python python/gv2_to_mega_bridge.py --gv2-port COM13 --mega-port COM7`.
3. Flash `arduino/gv2_to_mega/gv2_to_mega.ino` to the Mega and confirm it prints parsed detections over USB Serial.
4. Wire in your actual arm/motor control inside `routeDetection()` in the `.ino` file once calibration is validated.

## What is and is not verified

| Step | Status |
|---|---|
| Model structural validation | **Verified** (ran locally, see above) |
| Vela regression test (fresh export is worse) | **Verified** (ran locally, reproduced the exact warning) |
| Model flashes onto real GV2 | **NOT HARDWARE VERIFIED** |
| GV2 returns non-empty boxes on the real camera | **NOT HARDWARE VERIFIED** |
| Multi-object simultaneous detection on real scene | **NOT HARDWARE VERIFIED** |
| Arduino Mega receives and parses correctly | **NOT HARDWARE VERIFIED** |
| Robot coordinate mapping accuracy | **NOT HARDWARE VERIFIED** (depends on your physical calibration data) |
