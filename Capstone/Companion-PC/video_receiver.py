import socket
import cv2
import numpy as np
from ultralytics import YOLO

VIDEO_PORT = 11111

# Load YOLO model, downloads automatically first time
model = YOLO("yolov8n.pt")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", VIDEO_PORT))
sock.settimeout(3.0)

print("YOLO loaded successfully")
print(f"Listening for video on port {VIDEO_PORT}")

while True:
    try:
        data, addr = sock.recvfrom(65536)
        np_array = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        if frame is not None:
            # Run YOLO inference
            results = model(frame, verbose=False)

            # Draw bounding boxes on frame
            annotated_frame = results[0].plot()

            # Show annotated frame
            cv2.imshow("Drone Camera - YOLO", annotated_frame)

            # Print detections to terminal
            for box in results[0].boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls]
                print(f"Detected: {label} confidence: {conf:.2f}")

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except socket.timeout:
        print("Waiting for frames...")

sock.close()
cv2.destroyAllWindows()

# cd C:\Users\Sinan\Capstone\Companion-PC
# .\venv\Scripts\activate