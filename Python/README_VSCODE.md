# Running these in VS Code

## One-time setup

1. Open this `python/` folder (or the whole `BOTICS_ALPHA_AI/` folder) in VS Code.
2. Open a terminal in VS Code (`` Ctrl+` ``) and run:
   ```
   pip install -r ../requirements.txt
   ```
   (or just: `pip install pyserial opencv-python numpy` — that's all these files need)
3. Find your GV2's COM port: Windows Device Manager -> Ports (COM & LPT) -> note the number (e.g. `COM13`).
4. Open **`gv2_detect.py`** and edit the config block near the top:
   ```python
   COM_PORT = "COM13"        # <- change this to your actual port
   BAUD_RATE = 921600
   CONFIDENCE_THRESHOLD = 0.5
   ```
5. Do the same in `gv2_serial_test.py` if you use it.

## What to run for "connect to board, show video + coordinates"

**`gv2_detect.py`** is the one you want. Open it in VS Code and press the
**Run ▷** button (or right-click -> "Run Python File in Terminal").

With the defaults, it will:
- open the GV2 on the COM port you set,
- continuously ask GV2 to run inference **and** send back the camera image,
- pop up an OpenCV window showing the live camera feed with bounding boxes,
  class names, and confidence drawn on it,
- print every detection's `X, Y, W, H, CX, CY` to the VS Code terminal, live.

Press `q` with the video window focused to stop it, or `Ctrl+C` in the terminal.

## If something fails before you see video

Run **`gv2_serial_test.py`** first (also just press Run — defaults to
`--cmd invoke`) to check the board responds at all, with a much smaller
amount of code in the way. If that shows real JSON back from GV2 but
`gv2_detect.py` shows nothing, the problem is on the visualization side, not
the board connection. See `../docs/TROUBLESHOOTING.md`.

## File-by-file

| File | Run this when... |
|---|---|
| `gv2_serial_test.py` | You want a minimal, single-command diagnostic (is a model loaded? does invoke return anything?) |
| `gv2_detect.py` | **This is the one for live video + on-screen boxes + terminal coordinates.** |
| `calibration.py` | You're ready to map camera pixels to robot coordinates (needs `gv2_detect.py` numbers first) |
| `gv2_to_mega_bridge.py` | You're testing the Arduino Mega and need GV2's detections forwarded to it over a second COM port |

None of these load `best.pt` or run YOLO on your PC — they only talk to
GV2's on-device inference over serial, per the project's architecture
requirement.
