from ultralytics import YOLO
import cv2

# Load trained AI model
model = YOLO("best.pt")

# Open camera
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("ERROR: Camera open nahi hua")
    exit()

print("AI DETECTION + COORDINATES STARTED")
print("Press Q to quit")

while True:

    ret, frame = cap.read()

    if not ret:
        print("ERROR: Frame nahi mila")
        break

    # YOLO detection
    results = model.predict(
        source=frame,
        imgsz=224,
        conf=0.25,
        verbose=False
    )

    result = results[0]

    # Check every detected object
    for box in result.boxes:

        # Class
        cls_id = int(box.cls[0])
        class_name = model.names[cls_id]

        # Confidence
        confidence = float(box.conf[0])

        # Bounding box
        x1, y1, x2, y2 = box.xyxy[0].tolist()

        # CENTER X,Y
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        # Terminal output
        print(
            f"{class_name} | "
            f"conf={confidence:.2f} | "
            f"X={center_x} | Y={center_y}"
        )

        # Draw center point
        cv2.circle(
            frame,
            (center_x, center_y),
            5,
            (0, 0, 255),
            -1
        )

        # Draw coordinates
        cv2.putText(
            frame,
            f"{class_name} ({center_x},{center_y})",
            (center_x, center_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2
        )

    # Display
    annotated = result.plot()

    # Put our coordinate markers on top
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)

        cls_id = int(box.cls[0])
        name = model.names[cls_id]

        cv2.putText(
            annotated,
            f"{name}: ({cx},{cy})",
            (cx, cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2
        )

    cv2.imshow("BOTICS ALPHA - AI Detection", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
