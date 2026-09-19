# BOTICS_ALPHA_AI

> **Use `models_v2_yolov8n/` — this is the current, recommended model.**
> The original `models/` folder (YOLO26) is kept for reference only and is
> **deprecated**: it was root-caused to fail on real GV2 hardware because
> Seeed's own SenseCraft firmware does not support YOLO26 model metadata
> (confirmed on Seeed's forum, not guessed). See
> `docs/YOLOV8N_PIPELINE.md` for the full story, what changed, and what is
> and isn't validated yet.

Final AI deployment package for the BOTICS Alpha robotics competition system:

```
Sunny P5V04A CSI camera -> Grove Vision AI V2 (on-device inference)
    -> class + confidence + x/y/w/h -> UART -> Arduino Mega 2560
    -> coordinate mapping -> robotic arm -> pick/sort/place
```

The PC/Python code in this repo is for **debugging, calibration, and
visualization only**. The competition-time inference runs entirely on
Grove Vision AI V2's Ethos-U55 NPU.

## Directory structure

```
models_v2_yolov8n/   CURRENT / RECOMMENDED. YOLOv8n, imgsz=192, trained on
                     real recovered labels from your Seperate_Images.zip.
                       best.pt                              (PyTorch weights)
                       best_full_integer_quant_vela.tflite   (FLASH THIS ONE)
                       best_full_integer_quant_pre_vela.tflite (pre-Vela, for debugging)
                       labels.txt, metadata.json, vela_compile_summary.csv,
                       validation_confidence_report.json
models/              DEPRECATED. Original YOLO26 model + backup, kept for
                     reference. Root-caused as unsupported by SenseCraft/GV2
                     firmware (see docs/YOLOV8N_PIPELINE.md) — do not deploy.
dataset/             dataset_report.md (v1 findings) + yolo_dataset_v2/
                     (the real labeled dataset used to train the v2 model)
python/              gv2_serial_test.py, gv2_detect.py, calibration.py,
                     gv2_to_mega_bridge.py
arduino/             gv2_to_mega/gv2_to_mega.ino, xiao_gv2_bridge/xiao_gv2_bridge.ino
tools/               inspect_model.py, validate_tflite.py, validate_dataset.py,
                     build_dataset_v2.py, compare_tflite_vs_pt_v2.py
calibration/         calibration_points.json (template — fill via python/calibration.py)
docs/                YOLOV8N_PIPELINE.md (start here for the v2 model),
                     DEPLOYMENT.md, HARDWARE_CONNECTION.md, SERIAL_PROTOCOL.md,
                     TROUBLESHOOTING.md
requirements.txt
```

## Classes (fixed — do not reorder or rename)

```
0 red_patient
1 yellow_patient
2 green_patient
3 sample
```

## Start here

1. `docs/YOLOV8N_PIPELINE.md` — why the model changed, exactly how
   `models_v2_yolov8n/best_full_integer_quant_vela.tflite` was built and
   validated, and what is still unverified.
2. `docs/DEPLOYMENT.md` — flashing / running steps (still applicable; only
   the model file and its imgsz=192 input changed).
3. `docs/TROUBLESHOOTING.md` — the ranked, evidence-based checklist for the
   `boxes: []` problem the v1 model hit (root cause: YOLO26 unsupported by
   SenseCraft firmware, not something wrong with your flashing/wiring).

## Status summary

**Model**: `models_v2_yolov8n/best_full_integer_quant_vela.tflite` —
YOLOv8n, imgsz=192, INT8, NHWC, 100% NPU-offloaded (0 CPU-fallback ops,
clean Vela compile, no DEQUANTIZE/QUANTIZE warnings). Structurally validated
(7/7 checks) and confidence-tested against PyTorch ground truth on 60
held-out images (60/60 correct top-class agreement, 0/60 zero-detections).
**HARDWARE VALIDATION PENDING** — this file has not been flashed to or run
on your physical GV2; that is the next real test. See
`docs/YOLOV8N_PIPELINE.md` for full detail and honest caveats (dataset
generalization risk, on-device NMS not modeled by the offline test).

The v1 YOLO26 model in `models/` is **deprecated**: it compiled cleanly and
ran on the NPU at ~107ms, but returned `boxes: []` on every real-hardware
invocation because Seeed's SenseCraft firmware does not support YOLO26
metadata (confirmed on Seeed's own forum) — not an export or dataset defect.
