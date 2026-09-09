import socket
import cv2
import numpy as np
import json
from ultralytics import YOLO
import slam_client

# ── Real Tello addresses ─────────────────────────────────
DRONE_IP = "192.168.10.1"
COMMAND_PORT = 8889
VIDEO_PORT = 11111
DETECTION_PORT = 9999
UNITY_IP = "127.0.0.1"   # detections still forwarded to Unity for viz
# ─────────────────────────────────────────────────────────

model = YOLO("yolov8n.pt")

# Command socket: used once here just to start the video stream.
# If your control script already owns the command socket and is
# running in the same process, pass that socket in instead of
# creating a second one bound to the same local port.
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.settimeout(5.0)

video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_sock.bind(("0.0.0.0", VIDEO_PORT))
video_sock.settimeout(3.0)

detection_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def start_video_stream():
    cmd_sock.sendto(b"streamon", (DRONE_IP, COMMAND_PORT))
    try:
        resp, _ = cmd_sock.recvfrom(1024)
        print(f"streamon response: {resp.decode()}")
    except socket.timeout:
        print("No response to streamon, check connection")


start_video_stream()

print(f"Listening for video on port {VIDEO_PORT}")
print(f"Sending detections to Unity on port {DETECTION_PORT}")

# ── SLAM ──────────────────────────────────────────────────
# Start SLAM client, connects to mono_socket in WSL2
# Make sure mono_socket is running first in Ubuntu terminal:
# cd ~/ORB_SLAM3
# ./Examples/Monocular/mono_socket Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml
USE_SLAM = True
if USE_SLAM:
    try:
        slam_client.start()
        print("SLAM client connected")
    except Exception as e:
        print(f"SLAM connection failed: {e}, continuing without SLAM")
        USE_SLAM = False
# ─────────────────────────────────────────────────────────

while True:
    try:
        data, addr = video_sock.recvfrom(65536)
        np_array = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        if frame is not None:

            # ── SLAM ──────────────────────────────────────
            if USE_SLAM:
                slam_client.send_frame(frame)
                position = slam_client.get_position()
                if position["tracking"]:
                    print(f"SLAM pos: {position['x']:.2f}, "
                          f"{position['y']:.2f}, "
                          f"{position['z']:.2f}")
            # ─────────────────────────────────────────────

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
                    },
                    # ── SLAM position at time of detection ──
                    "slam_pos": slam_client.get_position() if USE_SLAM else None
                    # ────────────────────────────────────────
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

# Tell the drone to stop streaming before closing sockets
cmd_sock.sendto(b"streamoff", (DRONE_IP, COMMAND_PORT))
video_sock.close()
detection_sock.close()
cmd_sock.close()
cv2.destroyAllWindows()