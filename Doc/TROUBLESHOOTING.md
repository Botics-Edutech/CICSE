# Troubleshooting: `boxes: []`

Work through these **in order**. Each has a command that tells you which
bucket you're in — don't skip ahead based on a guess.

## 1. Is a valid model actually resident on GV2?

```
python python/gv2_serial_test.py --port COM13 --baud 921600 --cmd model
```
If `"size"` is `0` (this project saw exactly this earlier): the flash did
not take. Re-flash `models/best_full_integer_quant_vela.tflite` via
SenseCraft/Model Assistant and re-run this check before doing anything else.
This is the single most likely explanation given what was actually observed
on this project before, and it is also the cheapest to rule out.

## 2. Does the model file itself validate structurally?

```
python tools/validate_tflite.py models/best_full_integer_quant_vela.tflite
```
This should print all `PASS`. If it does not, stop — you have a file
problem, not a firmware/camera problem, and re-flashing won't help.

## 3. Is the camera actually delivering an image?

Run invoke WITH the image payload and confirm you get image bytes back:
```
python python/gv2_detect.py --port COM13 --baud 921600 --once
```
(omit `--result-only`). If `gv2_detect.py` never shows an OpenCV window /
prints a decode warning for the image field, the problem is upstream of
detection entirely — check the CSI ribbon orientation
(`docs/HARDWARE_CONNECTION.md`) before touching the model again.

## 4. Model loaded + camera working, but boxes still empty

At this point the remaining candidates, roughly in order of likelihood:

- **Confidence threshold set too high in firmware/SenseCraft config for this
  model.** Some flashing tools let you set a default score threshold
  separately from what your PC-side script requests. Check/lower it in
  SenseCraft, then retry `AT+INVOKE=1,0,1` directly (not through a script)
  and read the raw `perf`/`boxes` output.
- **Model registered with the wrong "algorithm type" tag.** SSCMA-Micro's
  on-device postprocessor is chosen partly by metadata associated with the
  model at flash time, not purely by inspecting the tflite file. If the
  custom-model upload flow asked you to pick an architecture/algorithm type,
  confirm it matches what actually produced this file (YOLO26 / a raw
  multi-scale detection head, 4 classes, 224 input) rather than a default
  guess. This is a firmware/tooling detail this environment cannot inspect
  without the actual SenseCraft session — **NOT HARDWARE VERIFIED**.
- **A genuine SSCMA firmware version incompatibility with YOLO26's specific
  head shape.** YOLO26 is a newer Ultralytics architecture (`reg_max: 1`, no
  DFL — confirmed via `tools/inspect_model.py --pt models/best.pt`, and this
  is YOLO26's *stock* configuration, not something unusual done to your
  model). If your GV2 firmware build predates full YOLO26 postprocessing
  support, this would explain consistently empty boxes despite everything
  else being correct. Check your SSCMA-Micro firmware version against
  Seeed's changelog for YOLO26 support, and update firmware if it's behind.

## 5. Getting some boxes, but multi-object detection is unreliable

This is a dataset-generalization issue, not a pipeline bug — see
`dataset/dataset_report.md`. The only images available for inspection
(`all_400_images_for_labeling.zip`) are single isolated objects on plain
backgrounds, materially easier than the cluttered multi-object competition
scene. If single-object detections work well but multi-object scenes with
the real mat background do not, that is the trigger to collect and label
proper multi-object scene photos and retrain — not before.

## 6. Recursive-Invoke bug (historical, already avoided in this codebase)

An earlier version of this project's Python called `Invoke()` from inside a
serial `on_connect`/`on_device_connect` callback, causing recursive/broken
behavior. `gv2_serial_test.py`, `gv2_detect.py`, and
`gv2_to_mega_bridge.py` in this repo all use a plain sequential
`open -> settle -> loop{invoke}` structure instead — if you modify them,
keep it that way.
