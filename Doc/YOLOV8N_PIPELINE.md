# v2: YOLOv8n pipeline (replaces the deprecated YOLO26 model)

## Why v1 (YOLO26) was replaced

The original `models/best.pt` / `models/best_full_integer_quant_vela.tflite`
was a **YOLO26n** model. It compiled cleanly with Vela and ran on GV2's
Ethos-U55 at ~107ms/inference, but produced `boxes: []` on every single
invocation, on real hardware, even at `confidence threshold = 1` across 60+
consecutive frames (see the device log the user supplied). That result ruled
out "just weak confidence" as the explanation.

After ruling out (with evidence, not guesses) bad flashing, wrong
preprocessing, and quantization collapse, the root cause was confirmed on
Seeed's own community forum: **Seeed staff (PJ_Glasso) stated that SenseCraft
does not currently support YOLO26 metadata.** The model's math was fine; the
firmware simply doesn't know how to parse a YOLO26 model's output/metadata
into detections. This is a supported-architecture problem, not a bug in the
exported file.

## What changed for v2

Seeed's own official Grove Vision AI V2 deployment documentation
(`wiki.seeedstudio.com/ma_deploy_yolov8/`) documents **YOLOv8n** as a
supported detection architecture for GV2/SenseCraft, with a specific,
documented pipeline:

```
yolo train detect model=yolov8n.pt data=./data.yaml imgsz=192
yolo export model=<best.pt> format=tflite imgsz=192 int8
vela --accelerator-config ethos-u55-64 <exported>.tflite
```

Note **imgsz=192**, not 224 — this is a deliberate, documented difference
from the v1 model, not an inconsistency.

## Dataset used for retraining

The user's originally-uploaded `all_400_images_for_labeling.zip` had zero
labels and single-object/plain-background images unsuitable for training a
combined multi-class detector (see `dataset/dataset_report.md`).

On a later upload, `Seperate_Images.zip` (4 separately-trained SenseCraft
single-class models, one per class) was found to contain, inside each
class's photo folder, a SenseCraft-exported `annotations.json` file with
**real pixel-space bounding boxes** collected during the user's own
SenseCraft training workflow — not synthetic or assumed labels. These were
missed on first inspection (only the raw `.jpg` files were seen) and
recovered on closer inspection.

`annotations.json` format (per entry): `{"img": "<base64 jpeg>", "rects":
[{"x": float, "y": float, "width": float, "height": float}], ...}` — direct
pixel coordinates, top-left origin, on the captured 224×224 image. This
coordinate convention (vs. percentage) was confirmed by a visual overlay
test, not assumed.

`build_dataset.py` (included) converts these into a single combined
YOLO-format dataset:
- 100 images per class × 4 classes = 400 total images, each with one real
  bounding box in the correct class slot
- 340 train / 60 val (85/15 split, seeded)
- A handful of source entries had a stray full-frame rect (an artifact of
  SenseCraft's live-tracking capture) alongside the real box; these were
  detected by area-fraction and dropped, keeping the real box only

This is a genuine improvement over v1's dataset situation — real, verified
per-image labels instead of no labels at all. **The same generalization
caveat as before still applies**: these are still single-object,
plain-background captures, not the cluttered multi-object competition
scene. Strong validation metrics on this val split reflect performance on
data resembling the training distribution, not a guarantee about the real
arena. See `models_v2_yolov8n/metadata.json` for the full caveat.

## Training

```python
from ultralytics import YOLO
m = YOLO('yolov8n.pt')
m.train(data='yolo_dataset/data.yaml', imgsz=192, epochs=80, batch=16,
        device='cpu', workers=2, patience=25)
```

80/80 epochs completed (patience=25, did not trigger early stop). Final
validation: precision=0.993, recall=1.0, mAP50=0.995, mAP50-95=0.964.

## Export toolchain pitfall (found and worked around in this session)

`ultralytics>=8.4.83` changed its default `format='tflite'` export backend
to a newer `litert_torch` pipeline. Exporting the v2 model with the
environment's installed `ultralytics==8.4.89` was tried first and produced:
- input shape `(1, 3, 192, 192)` — **NCHW**, not NHWC
- input/output dtype **float32** (with an internal quant/dequant wrapper),
  not direct INT8

This is the exact same bad-export pattern documented earlier in this
project for the v1 model's re-export attempts. It would very likely have
reproduced Vela's DEQUANTIZE/QUANTIZE CPU-fallback warnings and — separately
— an NCHW input tensor is not what Seeed's official pipeline (or GV2's
camera preprocessing) expects.

**Fix applied**: pinned `ultralytics==8.3.203` (last version before the
litert_torch switch) in an isolated Python virtual environment
(`export_venv/`, not committed — recreate with
`python3 -m venv export_venv && export_venv/bin/pip install ultralytics==8.3.203 tensorflow==2.16.2 "numpy<2"`),
which still uses the classic TensorFlow SavedModel → `TFLiteConverter`
export path. This produced, among several output variants, a
`best_full_integer_quant.tflite` with:
- input shape `(1, 192, 192, 3)` — **NHWC**, matching Seeed's documented
  pipeline
- input dtype **INT8**, scale=0.003875432536005974, zero_point=-128
- output shape `(1, 8, 756)` — INT8, scale=0.011811849661171436,
  zero_point=-128

This is the file that was Vela-compiled and delivered.

## Vela compile result

```
vela --accelerator-config ethos-u55-64 best_full_integer_quant.tflite
```

```
CPU operators = 0 (0.0%)
NPU operators = 261 (100.0%)
```

No DEQUANTIZE/QUANTIZE CPU-fallback warnings — a clean, fully
NPU-offloaded compile, the same signature the originally-provided v1 file
had (before it was found to be firmware-unsupported at the architecture
level, not the compile level). See `models_v2_yolov8n/vela_compile_summary.csv`.

## Offline validation performed (NOT hardware validation)

1. **Structural**: `tools/validate_tflite.py --imgsz 192 <file>` — input
   INT8/NHWC/quantized, output INT8/quantized/4-class-consistent, 100%
   NPU-offloaded. All 7 checks PASS.
2. **Confidence test**: every one of the 60 held-out validation images (15
   per class, never seen during training) was run through both PyTorch
   (`best.pt`, float, ground truth) and the pre-Vela INT8 TFLite (the exact
   INT8 arithmetic Vela compiles 1:1 — Vela schedules ops for the NPU, it
   does not change the math). Result: **60/60 images produced the correct
   top class from both**, at mean confidence 0.965 (PyTorch) and 0.725
   (INT8 TFLite — narrower dynamic range is expected from int8-quantizing
   the class-score tensor, not a defect). **0/60 produced zero detections**
   — a direct, evidence-based contrast with the v1 model's persistent
   `boxes: []` on real hardware.
   - One honest caveat from this test: with the on-device NMS/argmax not
     modeled by this offline script, nearly all 756 anchor positions per
     image cleared the 0.25 confidence threshold for the correct class on
     these simple, high-contrast, plain-background training images. This is
     expected for this dataset (very large, easy, dominant single object)
     and is exactly why on-device postprocessing does class-argmax + NMS
     across anchors, not naive thresholding. It also means a higher
     `confidence_threshold` (0.5–0.7) will likely give cleaner single-box
     output on real hardware than 0.25 — worth tuning during hardware
     bring-up, not a red flag in the model itself.
   - Full per-image results: `models_v2_yolov8n/validation_confidence_report.json`.

## What is explicitly NOT validated (HARDWARE VALIDATION PENDING)

- Actual execution of the Vela-compiled `ethos-u` command stream on real
  Ethos-U55 silicon — this environment cannot execute that custom op; it
  requires the physical GV2 board or Arm's Ethos-U simulator.
- SenseCraft/SSCMA firmware's on-device metadata parsing, anchor decode, and
  NMS for this specific model file.
- Whether `boxes` come back non-empty when this file is flashed to your GV2
  and invoked via `AT+INVOKE` / `AI.invoke()`.
- Everything downstream of that (XIAO bridge, Mega parsing, arm behavior).

Flash `models_v2_yolov8n/best_full_integer_quant_vela.tflite` to GV2 via
SenseCraft the same way as before, then re-run the exact diagnostic that
showed `boxes: []` last time (`python/gv2_serial_test.py --cmd invoke` or
the SenseCraft device logger) to get the first real hardware signal on this
architecture.
