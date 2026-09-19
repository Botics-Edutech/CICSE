# dataset/

This folder intentionally does **not** contain a copy of the original image
archives (`all_400_images_for_labeling.zip`, `Seperate Images.zip`) — they are
large and are your original uploads, kept untouched wherever you originally
stored them.

See `dataset_report.md` in this folder for the full inspection findings.

To reproduce the report yourself:

```bash
python ../tools/validate_dataset.py --images-zip /path/to/all_400_images_for_labeling.zip
```
