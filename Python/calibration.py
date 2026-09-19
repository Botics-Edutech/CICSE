#!/usr/bin/env python3
"""
calibration.py
===============
Camera-pixel -> robot-coordinate calibration for a camera mounted ON the
robot/robotic arm (not a fixed overhead camera). Because the camera moves
with the arm, this script does NOT assume any fixed geometry -- it builds an
empirical mapping from measured correspondences, same as you'd do for any
hand-eye / eye-in-hand calibration.

Method: 4+ point planar homography.
  You physically place the sample/patient object at N known robot (X, Y)
  positions (on the same working plane the arm will actually pick from),
  read off the object's detected pixel center (cx, cy) from GV2 at each
  position, and this script fits a homography H such that:

      [rx, ry, 1]^T ~ H * [px, py, 1]^T

  This is valid because the pick surface is planar and the object always
  sits at the same height on it -- a homography exactly captures perspective
  + lens effects for a single fixed plane. It does NOT generalize to a
  different height/plane; if the arm changes reach height, recalibrate.

  Camera-mounting caveat: because the camera rides on the arm, this
  calibration is only valid for the ONE arm pose (or fixed camera offset
  from the end effector) it was captured at. If your camera's position
  relative to the pick plane changes between shots (e.g. the arm moves to a
  different height/angle before it looks), you must either (a) always
  return to the same fixed "look" pose before invoking detection, or
  (b) calibrate separately for each fixed look-pose you use, or (c) do a
  full hand-eye calibration (not implemented here -- ask for it if needed;
  it requires the arm's forward kinematics, which weren't provided).

Usage:
  1. Collect points (interactive):
       python calibration.py collect --out calibration_points.json
     For each of >=4 points, place the object at a known robot (X,Y), read
     its pixel center from gv2_detect.py (or gv2_serial_test.py --cmd invoke),
     and enter both when prompted.

  2. Fit + save the homography:
       python calibration.py fit --in calibration_points.json --out homography.json

  3. Use it at runtime:
       from calibration import PixelToRobot
       mapper = PixelToRobot.load("homography.json")
       rx, ry = mapper.map(cx, cy)

Do NOT hard-code coordinates. Every number in homography.json comes from the
points you actually measured.
"""
import argparse
import json
import sys

import numpy as np


class PixelToRobot:
    def __init__(self, H: np.ndarray):
        self.H = H

    @classmethod
    def fit(cls, pixel_points, robot_points):
        """pixel_points, robot_points: lists of (x, y), same length, len>=4."""
        if len(pixel_points) < 4 or len(pixel_points) != len(robot_points):
            raise ValueError("Need >=4 matching (pixel, robot) point pairs.")
        px = np.array(pixel_points, dtype=np.float64)
        rx = np.array(robot_points, dtype=np.float64)
        H = _compute_homography(px, rx)
        return cls(H)

    def map(self, px: float, py: float):
        vec = np.array([px, py, 1.0])
        out = self.H @ vec
        out /= out[2]
        return float(out[0]), float(out[1])

    def save(self, path):
        with open(path, "w") as f:
            json.dump({"H": self.H.tolist()}, f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        return cls(np.array(d["H"]))

    def mean_reprojection_error(self, pixel_points, robot_points):
        errs = []
        for (px, py), (rx, ry) in zip(pixel_points, robot_points):
            mx, my = self.map(px, py)
            errs.append(((mx - rx) ** 2 + (my - ry) ** 2) ** 0.5)
        return float(np.mean(errs)), float(np.max(errs))


def _compute_homography(src, dst):
    """Direct Linear Transform (DLT) for a 2D homography, least-squares over
    all correspondences (works for exactly 4 or more points). No OpenCV
    dependency required, but we use it if available since it's more robust
    (RANSAC) with noisy real-world measurements."""
    try:
        import cv2
        H, _ = cv2.findHomography(src, dst, method=cv2.RANSAC if len(src) > 4 else 0)
        if H is not None:
            return H
    except ImportError:
        pass

    # Fallback: plain DLT least squares (no RANSAC / outlier rejection).
    A = []
    for (x, y), (u, v) in zip(src, dst):
        A.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        A.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    A = np.array(A)
    _, _, Vt = np.linalg.svd(A)
    H = Vt[-1].reshape(3, 3)
    return H / H[2, 2]


def collect_interactive(out_path):
    print("Collecting calibration points. Enter 'done' when finished (need >=4).")
    print("For each point: place the object at a known robot (X,Y) on the pick")
    print("plane, run detection, and note its reported pixel center (CX, CY).")
    pixel_points, robot_points = [], []
    while True:
        s = input(f"\nPoint {len(pixel_points) + 1} (or 'done'): ").strip()
        if s.lower() == "done":
            break
        try:
            px = float(input("  pixel CX: "))
            py = float(input("  pixel CY: "))
            rx = float(input("  robot X: "))
            ry = float(input("  robot Y: "))
        except ValueError:
            print("  invalid number, try again")
            continue
        pixel_points.append([px, py])
        robot_points.append([rx, ry])

    if len(pixel_points) < 4:
        print(f"Only {len(pixel_points)} points collected -- need at least 4. Not saving.")
        sys.exit(1)

    with open(out_path, "w") as f:
        json.dump({"pixel_points": pixel_points, "robot_points": robot_points}, f, indent=2)
    print(f"Saved {len(pixel_points)} points to {out_path}")


def fit_from_file(in_path, out_path):
    with open(in_path) as f:
        d = json.load(f)
    mapper = PixelToRobot.fit(d["pixel_points"], d["robot_points"])
    mean_err, max_err = mapper.mean_reprojection_error(d["pixel_points"], d["robot_points"])
    print(f"Fitted homography. Reprojection error: mean={mean_err:.2f} max={max_err:.2f} "
          f"(in robot-coordinate units).")
    if max_err > 5:  # heuristic sanity flag, not a hard rule -- adjust to your robot's units/tolerance
        print("WARNING: max reprojection error is large. Re-check your measured points, or the "
              "object may not sit exactly on the calibrated plane at every position.")
    mapper.save(out_path)
    print(f"Saved homography to {out_path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)

    c = sub.add_parser("collect")
    c.add_argument("--out", default="calibration_points.json")

    f = sub.add_parser("fit")
    f.add_argument("--in", dest="in_path", default="calibration_points.json")
    f.add_argument("--out", default="homography.json")

    args = ap.parse_args()
    if args.mode == "collect":
        collect_interactive(args.out)
    elif args.mode == "fit":
        fit_from_file(args.in_path, args.out)
