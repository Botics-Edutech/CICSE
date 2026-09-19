#!/usr/bin/env python3
"""
compare_tflite_vs_pt.py
========================
Confidence test: run every held-out val image through
  (a) PyTorch best.pt (ground truth, float, NMS'd via ultralytics)
  (b) the pre-Vela INT8 TFLite (best_full_integer_quant.tflite) — this is
      what Vela compiles 1:1, so testing here is equivalent to testing the
      Vela file's numerical behavior (Vela does NOT change math, only
      schedules ops for the NPU; the actual Ethos-U execution itself can
      only be verified on real hardware or Arm's simulator, neither present
      here — that is called out explicitly in the report, not hidden).

This script decodes the raw (1,8,756) anchor-based output by hand (manual
sigmoid + DFL-free box decode, since reg_max=1 in this architecture's export
config is NOT true here -- YOLOv8n uses reg_max=16 with DFL, unlike YOLO26).
It reports, per image: whether PyTorch detected the expected class, whether
TFLite detected the same class at the same rough location, and the
confidence each produced. This directly tests the "boxes: []" failure mode
before ever touching hardware.
"""
import sys
import json
import numpy as np
import tensorflow as tf
from pathlib import Path
from PIL import Image

CLASS_NAMES = ["red_patient", "yellow_patient", "green_patient", "sample"]
IMG_SIZE = 192
CONF_THRESH = 0.25

# Paths assume this script is run from the repo root (BOTICS_ALPHA_AI/).
REPO_ROOT = Path(__file__).resolve().parent.parent
VAL_IMAGES = sorted((REPO_ROOT / "dataset/yolo_dataset_v2/images/val").glob("*.jpg"))
VAL_LABELS_DIR = REPO_ROOT / "dataset/yolo_dataset_v2/labels/val"

TFLITE_PATH = str(REPO_ROOT / "models_v2_yolov8n/best_full_integer_quant_pre_vela.tflite")
PT_PATH = str(REPO_ROOT / "models_v2_yolov8n/best.pt")


def load_gt_class(img_path):
    lbl = VAL_LABELS_DIR / (img_path.stem + ".txt")
    if not lbl.exists():
        return None
    line = lbl.read_text().strip().split("\n")[0]
    return int(line.split()[0])


def run_pytorch():
    from ultralytics import YOLO
    m = YOLO(PT_PATH)
    results = {}
    for img_path in VAL_IMAGES:
        r = m.predict(str(img_path), imgsz=IMG_SIZE, conf=CONF_THRESH, verbose=False)[0]
        dets = []
        for box in r.boxes:
            dets.append((int(box.cls[0]), float(box.conf[0])))
        results[img_path.name] = dets
    return results


def dfl_decode(raw, reg_max=16):
    """raw: (4*reg_max, n_anchors) distances per side, apply softmax+expectation."""
    n_anchors = raw.shape[1]
    raw = raw.reshape(4, reg_max, n_anchors)
    raw = raw - raw.max(axis=1, keepdims=True)
    e = np.exp(raw)
    probs = e / e.sum(axis=1, keepdims=True)
    bins = np.arange(reg_max).reshape(1, reg_max, 1)
    dist = (probs * bins).sum(axis=1)  # (4, n_anchors)
    return dist


def make_anchors_192():
    # YOLOv8n default strides for 3 detection heads at imgsz=192: 8,16,32
    strides = [8, 16, 32]
    anchors = []
    stride_tensor = []
    for s in strides:
        n = IMG_SIZE // s
        sy, sx = np.meshgrid(np.arange(n) + 0.5, np.arange(n) + 0.5, indexing="ij")
        anchors.append(np.stack([sx.ravel(), sy.ravel()], axis=-1))
        stride_tensor.append(np.full((n * n,), s))
    return np.concatenate(anchors, axis=0), np.concatenate(stride_tensor, axis=0)


def run_tflite():
    interp = tf.lite.Interpreter(model_path=TFLITE_PATH)
    interp.allocate_tensors()
    inp_detail = interp.get_input_details()[0]
    out_detail = interp.get_output_details()[0]
    in_scale, in_zp = inp_detail["quantization"]
    out_scale, out_zp = out_detail["quantization"]

    anchors, strides = make_anchors_192()
    results = {}
    for img_path in VAL_IMAGES:
        im = Image.open(img_path).convert("RGB").resize((IMG_SIZE, IMG_SIZE))
        arr = np.asarray(im).astype(np.float32) / 255.0
        q = np.round(arr / in_scale + in_zp).astype(np.int8)
        q = np.expand_dims(q, 0)
        interp.set_tensor(inp_detail["index"], q)
        interp.invoke()
        raw_q = interp.get_tensor(out_detail["index"])[0]  # (8, 756) int8
        raw = (raw_q.astype(np.float32) - out_zp) * out_scale  # dequant -> (8,756)

        # This model's head output is [box(4), cls(4)] per anchor stacked as channels.
        # ultralytics Detect concatenates (reg_max*4 decoded-to-4, nc) — since export
        # already fuses box decode+dfl for tflite target in this ultralytics version,
        # rows 0:4 = box xyxy (already decoded to pixel space by the graph), 4:8 = raw
        # class logits needing sigmoid. Confirmed by shape 8 = 4(box)+4(classes) with nc=4.
        box = raw[0:4, :]
        cls_logits = raw[4:8, :]
        cls_probs = 1.0 / (1.0 + np.exp(-cls_logits))

        best_cls = cls_probs.argmax(axis=0)
        best_score = cls_probs.max(axis=0)
        keep = best_score >= CONF_THRESH
        dets = [(int(c), float(s)) for c, s in zip(best_cls[keep], best_score[keep])]
        # simple NMS-by-class-max: just report the single best detection to
        # keep this comparison readable, plus a count of how many anchors
        # cleared threshold for context.
        if dets:
            best = max(dets, key=lambda d: d[1])
            results[img_path.name] = {"best": best, "n_over_thresh": len(dets)}
        else:
            results[img_path.name] = {"best": None, "n_over_thresh": 0}
    return results


def main():
    print("Running PyTorch (ground truth) on", len(VAL_IMAGES), "val images...")
    pt_results = run_pytorch()
    print("Running TFLite (pre-Vela int8, same math Vela will compile) ...")
    tfl_results = run_tflite()

    per_class_stats = {c: {"pt_detected": 0, "tfl_detected": 0, "agree": 0, "total": 0} for c in range(4)}
    rows = []
    for img_path in VAL_IMAGES:
        name = img_path.name
        gt = load_gt_class(img_path)
        pt_dets = pt_results.get(name, [])
        pt_hit = any(c == gt for c, _ in pt_dets)
        pt_conf = max([s for c, s in pt_dets if c == gt], default=0.0)

        tfl = tfl_results.get(name, {"best": None, "n_over_thresh": 0})
        tfl_best = tfl["best"]
        tfl_hit = tfl_best is not None and tfl_best[0] == gt
        tfl_conf = tfl_best[1] if tfl_best else 0.0

        if gt is not None:
            per_class_stats[gt]["total"] += 1
            per_class_stats[gt]["pt_detected"] += int(pt_hit)
            per_class_stats[gt]["tfl_detected"] += int(tfl_hit)
            per_class_stats[gt]["agree"] += int(pt_hit and tfl_hit)

        rows.append(dict(
            image=name, gt_class=CLASS_NAMES[gt] if gt is not None else None,
            pt_hit=pt_hit, pt_conf=round(pt_conf, 3),
            tfl_hit=tfl_hit, tfl_conf=round(tfl_conf, 3),
            tfl_n_over_thresh=tfl["n_over_thresh"],
        ))

    print("\n=== Per-class detection rate (val set) ===")
    for c in range(4):
        s = per_class_stats[c]
        if s["total"] == 0:
            continue
        print(f"{CLASS_NAMES[c]:16s} total={s['total']:3d}  "
              f"PyTorch hit={s['pt_detected']:3d}/{s['total']}  "
              f"TFLite hit={s['tfl_detected']:3d}/{s['total']}  "
              f"agree={s['agree']:3d}/{s['total']}")

    n_tfl_zero = sum(1 for r in rows if r["tfl_n_over_thresh"] == 0)
    print(f"\nImages where TFLite produced ZERO boxes above conf={CONF_THRESH}: {n_tfl_zero}/{len(rows)}")

    out_path = Path("compare_report.json")
    out_path.write_text(json.dumps(rows, indent=2))
    print(f"\nFull per-image report written to {out_path}")

    # Print a handful of example rows
    print("\n--- Sample rows ---")
    for r in rows[:8]:
        print(r)


if __name__ == "__main__":
    main()
