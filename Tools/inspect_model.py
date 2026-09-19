#!/usr/bin/env python3
"""
inspect_model.py
=================
Inspects a best.pt (Ultralytics) checkpoint OR a .tflite model and prints a
structural report: input/output tensors, quantization, class mapping,
architecture head type.

This script does NOT execute Ethos-U-compiled ("ethos-u" custom op) TFLite
files -- that op can only run on real Ethos-U55 hardware or Arm's official
simulator, neither of which exists in a normal Python environment. For a
Vela-compiled file, this script parses the FlatBuffer directly instead of
calling the TFLite interpreter, so it still reports input/output shape,
dtype and quantization even though it cannot run inference.

Usage:
    python inspect_model.py --pt models/best.pt
    python inspect_model.py --tflite models/best_full_integer_quant_vela.tflite
"""
import argparse
import sys


def inspect_pt(path):
    import torch

    ckpt = torch.load(path, map_location="cpu", weights_only=False)
    print(f"=== {path} (PyTorch checkpoint) ===")
    print("ultralytics version used to train:", ckpt.get("version"))
    print("date:", ckpt.get("date"))
    ta = ckpt.get("train_args", {})
    print("base model:", ta.get("model"))
    print("imgsz:", ta.get("imgsz"))
    print("epochs:", ta.get("epochs"))
    print("data.yaml (training-time path, may not exist here):", ta.get("data"))
    m = ckpt["model"]
    print("task: detect | nc:", getattr(m, "nc", None))
    print("class names:", m.names)
    y = m.yaml if hasattr(m, "yaml") else {}
    print("end2end:", y.get("end2end"))
    print("reg_max:", y.get("reg_max"), "(1 = no DFL, direct box regression -- this is normal/stock for YOLO26)")
    print("scale:", y.get("scale"))
    tm = ckpt.get("train_metrics", {})
    if tm:
        print("--- reported validation metrics (measured on the training-time val split) ---")
        for k, v in tm.items():
            print(f"  {k}: {v}")
        print("  NOTE: these numbers only mean what the val split actually contained.")
        print("  If that split was single-object/plain-background images (see dataset_report.md),")
        print("  high mAP here does NOT guarantee good multi-object real-scene performance.")


def dump_tensor(sg, t):
    shape = [t.Shape(i) for i in range(t.ShapeLength())]
    q = t.Quantization()
    scale = zp = None
    if q is not None and q.ScaleLength() > 0:
        scale = [q.Scale(i) for i in range(q.ScaleLength())]
        zp = [q.ZeroPoint(i) for i in range(q.ZeroPointLength())]
    dtype_map = {0: "FLOAT32", 9: "INT8", 3: "UINT8", 2: "INT32", 10: "INT16"}
    return dict(
        name=t.Name().decode() if t.Name() else None,
        shape=shape,
        dtype=dtype_map.get(t.Type(), t.Type()),
        scale=scale,
        zero_point=zp,
    )


def inspect_tflite(path):
    from tensorflow.lite.python import schema_py_generated as schema_fb

    with open(path, "rb") as f:
        buf = f.read()
    model = schema_fb.Model.GetRootAsModel(buf, 0)
    sg = model.Subgraphs(0)

    print(f"=== {path} (TFLite FlatBuffer, parsed directly) ===")
    print("Schema version:", model.Version())
    print("Num subgraphs:", model.SubgraphsLength())

    print("--- Operator codes ---")
    is_vela = False
    for i in range(model.OperatorCodesLength()):
        oc = model.OperatorCodes(i)
        custom = oc.CustomCode()
        custom_str = custom.decode() if custom else None
        if custom_str == "ethos-u":
            is_vela = True
        print(f"  [{i}] builtin={oc.BuiltinCode()} custom={custom_str}")

    print("--- Inputs ---")
    for i in range(sg.InputsLength()):
        idx = sg.Inputs(i)
        print(" ", dump_tensor(sg, sg.Tensors(idx)))

    print("--- Outputs ---")
    for i in range(sg.OutputsLength()):
        idx = sg.Outputs(i)
        t = dump_tensor(sg, sg.Tensors(idx))
        print(" ", t)
        shape = t["shape"]
        if len(shape) == 3 and shape[1] not in (None,):
            c, n = shape[1], shape[2]
            nc = c - 4
            print(f"    -> interpretation: {c} channels = 4 bbox coords + {nc} classes; "
                  f"{n} anchors is a RAW multi-scale detection head (needs external "
                  f"decode + NMS), consistent with YOLO26/YOLOv8-style export WITHOUT "
                  f"end-to-end NMS baked in.")

    print("--- Ops in subgraph ---")
    for i in range(sg.OperatorsLength()):
        op = sg.Operators(i)
        ins = [op.Inputs(j) for j in range(op.InputsLength())]
        outs = [op.Outputs(j) for j in range(op.OutputsLength())]
        print(f"  op[{i}] opcode_index={op.OpcodeIndex()} inputs={ins} outputs={outs}")

    n_ops = sg.OperatorsLength()
    if is_vela:
        if n_ops == 1:
            print("\nVERDICT: single opaque 'ethos-u' op, no separate CPU-side TFLite ops.")
            print("This means 100% of the graph was placed on the Ethos-U NPU at compile")
            print("time -- no CPU fallback ops (e.g. no stray DEQUANTIZE/QUANTIZE wrapper),")
            print("which is the ideal, cleanest possible Vela compile result.")
        else:
            print(f"\nVERDICT: {n_ops} ops present alongside 'ethos-u' -- some ops fell back")
            print("to CPU. Inspect which ones; CPU-side DEQUANTIZE/QUANTIZE wrappers here")
            print("usually mean the float32 in/out boundary was not fully quantized before")
            print("Vela compilation (see docs/TROUBLESHOOTING.md).")
        print("\nNOTE: actual numerical correctness of the compiled Ethos-U command stream")
        print("cannot be verified without running it on real Ethos-U55 hardware (the GV2")
        print("board) or Arm's Ethos-U simulator. This script only confirms structure.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--pt", help="path to a .pt checkpoint")
    ap.add_argument("--tflite", help="path to a .tflite file")
    args = ap.parse_args()
    if not args.pt and not args.tflite:
        ap.error("pass --pt and/or --tflite")
    if args.pt:
        inspect_pt(args.pt)
    if args.tflite:
        inspect_tflite(args.tflite)
