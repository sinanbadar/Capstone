import cv2
import json
import socket
import platform
from ultralytics import YOLO
from djitellopy import Tello

DETECTION_PORT = 9999
UNITY_IP = "127.0.0.1"
DRONE_CONTROLLER_PORT = 9996

# ── SLAM ─────────────────────────────────────────────────
USE_SLAM = True
if USE_SLAM:
    import slam_client
    try:
        slam_client.start()
        print("SLAM client connected")
    except Exception as e:
        print(f"SLAM connection failed: {e}, continuing without SLAM")
        USE_SLAM = False
# ─────────────────────────────────────────────────────────

model = YOLO("yolov8n.pt")
detection_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone_controller_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

drone = Tello(host="192.168.10.1")
drone.connect()
print(f"Battery: {drone.get_battery()}%")
drone.streamon()

frame_reader = drone.get_frame_read()
print("Stream started, running YOLO")

while True:
    frame = frame_reader.frame
    if frame is None:
        continue

    # ── SLAM ──────────────────────────────────────────────
    if USE_SLAM:
        slam_client.send_frame(frame)
        position = slam_client.get_position()
        if position["tracking"]:
            print(f"SLAM pos: {position['x']:.2f}, "
                  f"{position['y']:.2f}, "
                  f"{position['z']:.2f}")
    # ─────────────────────────────────────────────────────

    results = model(frame, verbose=False)
    annotated = results[0].plot()
    cv2.imshow("Tello YOLO", annotated)

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
            },
            "slam_pos": slam_client.get_position() if USE_SLAM else None
        }
        detections.append(detection)
        print(f"Detected: {label} {conf:.2f}")

    if detections:
        message = json.dumps(detections).encode()
        # Send to Unity AR overlay
        detection_sock.sendto(message, (UNITY_IP, DETECTION_PORT))
        # Send to drone controller for autonomous detection response
        drone_controller_sock.sendto(message, ("127.0.0.1", DRONE_CONTROLLER_PORT))

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

drone.streamoff()
detection_sock.close()
drone_controller_sock.close()
cv2.destroyAllWindows()