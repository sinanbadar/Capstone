import socket
import cv2
import numpy as np
import json
from ultralytics import YOLO

VIDEO_PORT = 11111
DETECTION_PORT = 9999
UNITY_IP = "127.0.0.1"

model = YOLO("yolov8n.pt")

video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_sock.bind(("0.0.0.0", VIDEO_PORT))
video_sock.settimeout(3.0)

detection_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Listening for video on port {VIDEO_PORT}")
print(f"Sending detections to Unity on port {DETECTION_PORT}")

while True:
    try:
        data, addr = video_sock.recvfrom(65536)
        np_array = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        if frame is not None:
            results = model(frame, verbose=False)
            annotated_frame = results[0].plot()
            cv2.imshow("Drone Camera - YOLO", annotated_frame)

            detections = []
            for box in results[0].boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls]
                xyxy = box.xyxy[0].tolist()

                detection = {
                    "label": label,
                    "confidence": round(conf, 2),
                    "bbox": {
                        "x1": round(xyxy[0]),
                        "y1": round(xyxy[1]),
                        "x2": round(xyxy[2]),
                        "y2": round(xyxy[3])
                    }
                }
                detections.append(detection)
                print(f"Detected: {label} confidence: {conf:.2f}")

            if detections:
                message = json.dumps(detections).encode()
                detection_sock.sendto(message, (UNITY_IP, DETECTION_PORT))

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except socket.timeout:
        print("Waiting for frames...")

video_sock.close()
detection_sock.close()
cv2.destroyAllWindows()