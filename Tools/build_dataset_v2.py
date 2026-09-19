#!/usr/bin/env python3
"""
Build a real YOLO-format labeled dataset from SenseCraft's exported
annotations.json files (found inside Seperate_Images.zip's per-class
folders). Each entry has a base64-embedded image + one or more pixel-space
rects (x, y, width, height, top-left origin). A stray full-frame rect
([0,0,224,224]-ish, an artifact of SenseCraft's live tracking) is dropped
when a smaller, real rect is also present.

This is the exact script used to build dataset/yolo_dataset_v2/ for
reproducibility. To re-run it: unzip your Seperate_Images.zip so that
extracted/sep_photos/{red,yellow,green,sample}/annotations.json exist
relative to wherever you run this from, then `python3 build_dataset_v2.py`.
The already-built dataset is shipped at dataset/yolo_dataset_v2/, so you do
NOT need to re-run this to use the delivered model.
"""
import json
import base64
import io
import random
from pathlib import Path
from PIL import Image

random.seed(42)

SOURCES = {
    0: "extracted/sep_photos/red/annotations.json",       # red_patient
    1: "extracted/sep_photos/yellow/annotations.json",    # yellow_patient
    2: "extracted/sep_photos/green/annotations.json",     # green_patient
    3: "extracted/sep_photos/sample/annotations.json",    # sample
}
CLASS_NAMES = ["red_patient", "yellow_patient", "green_patient", "sample"]

OUT = Path("yolo_dataset")
VAL_FRACTION = 0.15


def is_full_frame(rect, img_w, img_h, thresh=0.85):
    area_frac = (rect["width"] * rect["height"]) / (img_w * img_h)
    return area_frac > thresh


def main():
    stats = {c: 0 for c in CLASS_NAMES}
    dropped_multi = 0

    for class_id, path in SOURCES.items():
        d = json.load(open(path))
        entries = d["annotations"]
        n_val = max(1, int(len(entries) * VAL_FRACTION))
        val_ids = set(random.sample(range(len(entries)), n_val))

        for i, item in enumerate(entries):
            img_bytes = base64.b64decode(item["img"])
            im = Image.open(io.BytesIO(img_bytes)).convert("RGB")
            w, h = im.size

            rects = item["rects"]
            if len(rects) > 1:
                # drop any rect that's basically the whole frame
                real_rects = [r for r in rects if not is_full_frame(r, w, h)]
                if len(real_rects) == 1:
                    rects = real_rects
                else:
                    dropped_multi += 1
                    rects = [max(rects, key=lambda r: r["width"] * r["height"])]
                    if is_full_frame(rects[0], w, h):
                        continue  # no usable real box for this image, skip it

            split = "val" if i in val_ids else "train"
            fname = f"{CLASS_NAMES[class_id]}_{item['id']}"
            img_path = OUT / "images" / split / f"{fname}.jpg"
            lbl_path = OUT / "labels" / split / f"{fname}.txt"
            im.save(img_path, quality=95)

            lines = []
            for r in rects:
                x, y, rw, rh = r["x"], r["y"], r["width"], r["height"]
                # clip to image bounds
                x = max(0, min(x, w))
                y = max(0, min(y, h))
                rw = max(1, min(rw, w - x))
                rh = max(1, min(rh, h - y))
                cx = (x + rw / 2) / w
                cy = (y + rh / 2) / h
                nw = rw / w
                nh = rh / h
                lines.append(f"{class_id} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}")
            lbl_path.write_text("\n".join(lines) + "\n")
            stats[CLASS_NAMES[class_id]] += 1

    print("Images written per class:", stats)
    print("Multi-rect images with no clean single box (skipped or reduced):", dropped_multi)

    n_train = len(list((OUT / "images" / "train").glob("*.jpg")))
    n_val = len(list((OUT / "images" / "val").glob("*.jpg")))
    print(f"train={n_train} val={n_val}")

    yaml_content = f"""path: {OUT.resolve()}
train: images/train
val: images/val
nc: 4
names: {CLASS_NAMES}
"""
    (OUT / "data.yaml").write_text(yaml_content)
    print("Wrote", OUT / "data.yaml")


if __name__ == "__main__":
    main()
