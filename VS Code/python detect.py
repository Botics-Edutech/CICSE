import base64
import time
import cv2
import numpy as np
from ultralytics import YOLO
from sscma.micro.client import SerialClient
from sscma.micro.device import Device

# ---------------- CONFIG (apne hisaab se change karo) ----------------
GV2_PORT = "COM13"        # Device Manager se GV2 ka COM port
MODEL_PATH = "best.pt"   # tumhara trained model
CONFIDENCE_THRESHOLD = 0.5
# -----------------------------------------------------------------

model = YOLO(MODEL_PATH)
latest_frame = {"img": None}


def monitor_handler(msg):
    # GV2 har frame ke saath yeh function call karta hai
    if "image" in msg:
        jpeg_bytes = base64.b64decode(msg["image"])
        nparr = np.frombuffer(jpeg_bytes, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        latest_frame["img"] = img


def on_device_connect(device):
    print("GV2 connected. Starting continuous capture...")
    device.invoke(-1, False, True)  # -1 = continuous frames, True = image chahiye


def main():
    client = SerialClient(GV2_PORT)
    device = Device(client)
    device.on_monitor = monitor_handler
    device.on_connect = on_device_connect
    device.loop_start()

    print("AI DETECTION + COORDINATES STARTED")
    print("Press Q to quit")

    try:
        while True:
            frame = latest_frame["img"]
            if frame is None:
                time.sleep(0.05)
                continue

            results = model.predict(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)[0]

            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]

                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                w, h = x2 - x1, y2 - y1

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{cls_name} {conf:.2f}", (x1, max(y1 - 8, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                print(f"Class: {cls_name} | Conf: {conf:.2f} | X:{cx} Y:{cy} W:{w} H:{h}")

            cv2.imshow("GV2 Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        device.loop_stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
