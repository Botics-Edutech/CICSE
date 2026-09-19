#!/usr/bin/env python3
"""
validate_dataset.py
====================
Read-only inspection of all_400_images_for_labeling.zip (and, if present,
Seperate Images.zip). Never modifies the original archives.

Usage:
    python validate_dataset.py --images-zip all_400_images_for_labeling.zip
"""
import argparse
import zipfile
from collections import Counter
from pathlib import Path


def classify_prefix(name):
    stem = Path(name).stem.lower()
    for cls in ["red", "yellow", "green", "sample"]:
        if stem.startswith(cls):
            return cls
    return "unknown"


def main(images_zip):
    print(f"Inspecting: {images_zip} (read-only, not extracted permanently)")
    with zipfile.ZipFile(images_zip) as z:
        names = [n for n in z.namelist() if not n.endswith("/")]
        images = [n for n in names if n.lower().endswith((".jpg", ".jpeg", ".png"))]
        labels = [n for n in names if n.lower().endswith(".txt")]
        yamls = [n for n in names if n.lower().endswith(".yaml") or n.lower().endswith(".yml")]

        print(f"Total files: {len(names)}")
        print(f"Image files: {len(images)}")
        print(f"YOLO .txt label files found: {len(labels)}")
        print(f"data.yaml / .yml files found: {len(yamls)}")

        by_class = Counter(classify_prefix(n) for n in images)
        print("\nImage count by filename-prefix class:")
        for cls, count in sorted(by_class.items()):
            print(f"  {cls:10s}: {count}")

        print("\n--- Verdict ---")
        if len(labels) == 0:
            print("NO YOLO label files (.txt) exist in this archive.")
            print("This archive is RAW, UNLABELED image data -- consistent with its name")
            print("'all_400_images_for_labeling.zip'. It cannot be used to verify the")
            print("labels/boxes that were actually used to train best.pt, because those")
            print("labels are not present here (best.pt's train_args points at a Colab")
            print("path '/content/custom_data/custom_data/data.yaml' that was never")
            print("uploaded). Do NOT invent labels for this data.")
        balanced = len(set(by_class.values())) <= 1
        print(f"Class balance by filename: {'balanced' if balanced else 'IMBALANCED'} "
              f"({dict(by_class)})")

        # Sample a few images to check resolution / single-vs-multi-object framing.
        try:
            from PIL import Image
            import io
            sample_names = images[:5]
            print("\nSample image dimensions (first 5):")
            for n in sample_names:
                data = z.read(n)
                im = Image.open(io.BytesIO(data))
                print(f"  {n}: {im.size} {im.mode}")
        except ImportError:
            print("(Pillow not installed -- skipping per-image dimension check)")

    print("\nManual visual inspection of sample images (done separately) showed:")
    print("  - each image is a SINGLE isolated object")
    print("  - plain white/gray background, no clutter, no other classes in frame")
    print("  - already pre-cropped/resized to 224x224")
    print("This is a materially EASIER distribution than the real competition scene")
    print("(multiple objects together, patterned mat background, first-aid box in")
    print("frame). High val mAP on this kind of data does not guarantee equivalent")
    print("real-scene multi-object performance. See docs/TROUBLESHOOTING.md.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--images-zip", required=True)
    args = ap.parse_args()
    main(args.images_zip)
