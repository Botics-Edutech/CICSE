# Serial Protocol Reference

## 1. GV2 <-> PC : SSCMA-Micro AT protocol (real, verified)

Source: [Seeed-Studio/SSCMA-Micro `docs/protocol/at_protocol.md` (1.0.x branch)](https://github.com/Seeed-Studio/SSCMA-Micro/blob/1.0.x/docs/protocol/at_protocol.md).
Everything in this section is quoted/derived from that document, not guessed.

**Framing**
- Command sent to device: `<COMMAND>\r`
- Response from device: `\r<json>\n`
- A single `AT+INVOKE` can produce **two** JSON lines: a `"type":0` operation
  reply first, then a `"type":1` event reply carrying the actual result.

**Commands used in this project**

| Command | Purpose |
|---|---|
| `AT+MODEL?` | Query which model is resident and its size |
| `AT+INFO?` | Device info / CRC |
| `AT+INVOKE=1,0,1` | Run one inference (`N_TIMES=1`, `DIFFERED=0`, `RESULT_ONLY=1` — no image payload, lower latency) |
| `AT+INVOKE=1,0,0` | Run one inference and include the image (needed for OpenCV preview) |

**`AT+MODEL?` response**
```json
{"type":0,"name":"MODEL?","code":0,"data":{"id":2,"type":3,"address":5242880,"size":267024}}
```
`size == 0` means no valid model is currently resident in that slot — a
flashing problem, not a model/inference problem. This project saw exactly
this symptom earlier.

**`AT+INVOKE` event response**
```json
{"type":1,"name":"INVOKE","code":0,"data":{"count":8,"perf":[8,365,0],"boxes":[[87,83,77,65,70,0]]}}
```
- `perf`: `[preprocess_ms, inference_ms, postprocess_ms]` (approximate — confirm against your firmware's actual semantics if it matters for your latency budget)
- `boxes`: array of `[x, y, w, h, score, target_id]`
  - `x, y, w, h`: pixel-space box (top-left x/y, width, height)
  - `score`: 0-100 integer confidence (this project's Python code divides by 100)
  - `target_id`: maps to this project's fixed class order — `0=red_patient, 1=yellow_patient, 2=green_patient, 3=sample`

An empty `boxes: []` with `code: 0` means inference genuinely ran but nothing
cleared the on-device decode/threshold — see `docs/TROUBLESHOOTING.md`.

## 1b. GV2 <-> XIAO : Seeed_Arduino_SSCMA library (real, verified API)

When a XIAO board is seated in GV2's onboard expansion header (the
competition setup — see `docs/HARDWARE_CONNECTION.md`), use Seeed's own
`Seeed_Arduino_SSCMA` Arduino library instead of hand-parsing AT-command
JSON. This is confirmed working, real API (from a published Seeed community
XIAO ESP32-S3 + GV2 project, not guessed):

```cpp
HardwareSerial atSerial(0);
SSCMA AI;
AI.begin(&atSerial);
...
AI.invoke(1, false, true);          // == AT+INVOKE=1,0,1
for (int i = 0; i < AI.boxes().size(); i++) {
  AI.boxes()[i].x / .y / .w / .h / .score / .target
}
```

`AI.invoke(1, false, true)`'s three arguments are the same
`N_TIMES, DIFFERED, RESULT_ONLY` parameters as the raw AT command in section
1 above. See `arduino/xiao_gv2_bridge/xiao_gv2_bridge.ino` for the full
sketch built on this.

## 2. XIAO -> Arduino Mega : bridge line protocol

The Mega cannot practically parse the raw SSCMA JSON directly (limited RAM,
no robust JSON library assumed installed) — see `docs/HARDWARE_CONNECTION.md`
and the header comment in `arduino/gv2_to_mega/gv2_to_mega.ino` for the
reasoning. Instead, whatever sits between GV2 and the Mega sends this fixed,
fast-to-parse ASCII line:

```
D,<class_id>,<confidence_x100>,<x>,<y>,<w>,<h>,<cx_x10>,<cy_x10>\n
```

Example: `D,0,94,87,83,77,65,1255,1155\n` decodes to `red_patient`,
confidence `0.94`, box `(87,83,77,65)`, center `(125.5, 115.5)`.

**Two things emit this exact same line — pick whichever matches what you're doing:**

| Emitter | When to use |
|---|---|
| `arduino/xiao_gv2_bridge/xiao_gv2_bridge.ino` (runs on the XIAO seated in GV2's header) | **Competition setup** — no PC involved |
| `python/gv2_to_mega_bridge.py` (runs on a laptop between GV2's USB-C and a second COM port to the Mega) | **Development/testing** — when you want to watch/debug on a PC before wiring up the XIAO |

`gv2_to_mega.ino` on the Mega reads `Serial1` and doesn't know or care which
one sent the line — no Mega-side changes needed when you switch from one to
the other.
