#!/usr/bin/env python3
"""
validate_tflite.py
===================
Structural validation for the GV2-deployable TFLite model, plus (optionally)
an end-to-end re-export + Vela regression check against the current toolchain,
so you can see whether re-exporting from best.pt with today's `ultralytics`
would make things better or worse before you ever touch the real file.

Checks performed (see report printed at the end):
  1. Input: shape, dtype, quantization scale/zero-point, layout (NHWC vs NCHW)
  2. Output: shape, dtype, quantization scale/zero-point
  3. Class count consistency vs the 4 required classes
  4. Op-level structure: is it a clean single 'ethos-u' op (100% NPU) or does
     it carry CPU-fallback ops (DEQUANTIZE/QUANTIZE wrappers)?
  5. (optional, --compare-fresh-export) re-exports best.pt with the ultralytics
     version installed in THIS environment and Vela-compiles it, to show
     concretely whether that pipeline reproduces the known-bad
     DEQUANTIZE/QUANTIZE warning pattern.

This script cannot execute the Ethos-U command stream itself (needs real
Ethos-U55 hardware or Arm's simulator), so "PASS" here means
"structurally sound for Ethos-U/GV2 deployment", not "produces correct boxes
on the real device". That last step is only provable on hardware -- see
docs/DEPLOYMENT.md and docs/TROUBLESHOOTING.md.
"""
import argparse
import subprocess
import sys
from pathlib import Path

REQUIRED_CLASSES = ["red_patient", "yellow_patient", "green_patient", "sample"]


def load_tensors(path):
    from tensorflow.lite.python import schema_py_generated as schema_fb

    with open(path, "rb") as f:
        buf = f.read()
    model = schema_fb.Model.GetRootAsModel(buf, 0)
    sg = model.Subgraphs(0)

    def dump(t):
        shape = [t.Shape(i) for i in range(t.ShapeLength())]
        q = t.Quantization()
        scale = zp = None
        if q is not None and q.ScaleLength() > 0:
            scale = [q.Scale(i) for i in range(q.ScaleLength())]
            zp = [q.ZeroPoint(i) for i in range(q.ZeroPointLength())]
        dtype_map = {0: "FLOAT32", 9: "INT8", 3: "UINT8"}
        return dict(shape=shape, dtype=dtype_map.get(t.Type(), t.Type()), scale=scale, zero_point=zp)

    ops = []
    op_codes = []
    for i in range(model.OperatorCodesLength()):
        oc = model.OperatorCodes(i)
        custom = oc.CustomCode()
        op_codes.append(custom.decode() if custom else f"builtin:{oc.BuiltinCode()}")
    for i in range(sg.OperatorsLength()):
        op = sg.Operators(i)
        ops.append(op_codes[op.OpcodeIndex()])

    inputs = [dump(sg.Tensors(sg.Inputs(i))) for i in range(sg.InputsLength())]
    outputs = [dump(sg.Tensors(sg.Outputs(i))) for i in range(sg.OutputsLength())]
    return inputs, outputs, ops


def validate(path, imgsz=224):
    print(f"\n{'=' * 60}\nValidating: {path}\n{'=' * 60}")
    checks = {}
    inputs, outputs, ops = load_tensors(path)

    inp = inputs[0]
    print("INPUT :", inp)
    checks["input_is_int8"] = inp["dtype"] == "INT8"
    checks[f"input_shape_{imgsz}_nhwc"] = inp["shape"] == [1, imgsz, imgsz, 3]
    checks["input_quant_present"] = inp["scale"] is not None

    out = outputs[0]
    print("OUTPUT:", out)
    checks["output_is_int8"] = out["dtype"] == "INT8"
    checks["output_quant_present"] = out["scale"] is not None
    if len(out["shape"]) == 3:
        c = out["shape"][1]
        nc = c - 4
        checks["class_count_matches_4"] = nc == len(REQUIRED_CLASSES)
        print(f"Derived class count: {nc} (expected {len(REQUIRED_CLASSES)}: {REQUIRED_CLASSES})")

    n_cpu_ops = sum(1 for o in ops if o != "ethos-u")
    checks["fully_npu_offloaded"] = ("ethos-u" in ops) and (n_cpu_ops == 0)
    print(f"Ops: {ops}  (CPU-fallback ops: {n_cpu_ops})")

    print("\n--- Checklist ---")
    all_ok = True
    for k, v in checks.items():
        print(f"  [{'PASS' if v else 'FAIL'}] {k}")
        all_ok = all_ok and v
    print(f"\nStructural validation: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def compare_fresh_export(pt_path, calib_yaml, out_dir):
    """Re-export best.pt with the ultralytics version installed HERE and
    Vela-compile it, purely to demonstrate whether today's default toolchain
    regresses vs the provided file. This does NOT overwrite anything."""
    print(f"\n{'=' * 60}\nCOMPARISON: fresh export from {pt_path} with current toolchain\n{'=' * 60}")
    from ultralytics import YOLO

    m = YOLO(pt_path)
    fresh_path = m.export(format="tflite", int8=True, imgsz=224, data=calib_yaml)
    print("Fresh pre-Vela export:", fresh_path)
    validate(fresh_path)

    print("\nRunning Vela on the fresh export to check for warnings...")
    r = subprocess.run(
        ["vela", "--accelerator-config", "ethos-u55-64", "--output-dir", out_dir, str(fresh_path)],
        capture_output=True, text=True,
    )
    print(r.stdout)
    if "DEQUANTIZE" in r.stdout or "QUANTIZE" in r.stdout:
        print("*** REPRODUCED the DEQUANTIZE/QUANTIZE warning pattern with today's toolchain. ***")
        print("*** This confirms: do NOT re-export with this environment's ultralytics/litert_torch ***")
        print("*** pipeline -- keep the originally-provided best_full_integer_quant_vela.tflite.    ***")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("tflite", help="path to the deployable .tflite to validate")
    ap.add_argument("--compare-fresh-export", action="store_true",
                     help="also re-export best.pt with the current toolchain and Vela-compile it for comparison")
    ap.add_argument("--pt", default="models/best.pt")
    ap.add_argument("--calib-yaml", default=None, help="data.yaml used only for INT8 calibration images")
    ap.add_argument("--out-dir", default="tools/_vela_compare_out")
    ap.add_argument("--imgsz", type=int, default=224,
                     help="expected square input resolution (224 for the original YOLO26 pipeline, 192 for the YOLOv8n/Seeed-documented pipeline)")
    args = ap.parse_args()

    ok = validate(args.tflite, imgsz=args.imgsz)

    if args.compare_fresh_export:
        if not args.calib_yaml:
            print("\n--compare-fresh-export requires --calib-yaml pointing at a data.yaml "
                  "with at least images (labels not required for INT8 calibration).")
            sys.exit(1)
        compare_fresh_export(args.pt, args.calib_yaml, args.out_dir)

    sys.exit(0 if ok else 1)
