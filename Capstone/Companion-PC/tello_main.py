import cv2
import json
import math
import socket
import time
import threading
from djitellopy import Tello
from ultralytics import YOLO
from controller_input import ControllerInput, ControlMode
from autonomous_search import AutonomousSearch

# ── CONFIG ───────────────────────────────────────────────
DRONE_IP = "192.168.10.1"
UNITY_IP = "127.0.0.1"
DETECTION_PORT = 9999
DETECTION_LISTEN_PORT = 9996
# ─────────────────────────────────────────────────────────

# ── SLAM ─────────────────────────────────────────────────
USE_SLAM = False
if USE_SLAM:
    import slam_client
    try:
        slam_client.start()
        print("SLAM client connected")
    except Exception as e:
        print(f"SLAM failed: {e}")
        USE_SLAM = False
# ─────────────────────────────────────────────────────────

# ── DRONE ────────────────────────────────────────────────
drone = Tello(host=DRONE_IP)
drone.connect()
print(f"Battery: {drone.get_battery()}%")
drone.streamon()
frame_reader = drone.get_frame_read()
# ─────────────────────────────────────────────────────────

model = YOLO("yolov8n.pt")
detection_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone_controller_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
controller = ControllerInput(unity_ip=UNITY_IP)

# ── DETECTION AND FRAME SHARING ───────────────────────────
latest_detections = []
detection_lock = threading.Lock()
latest_frame = None
frame_lock = threading.Lock()

def get_latest_detections():
    with detection_lock:
        return latest_detections.copy()
# ─────────────────────────────────────────────────────────

def get_position():
    if USE_SLAM:
        return slam_client.get_position()
    return {"x": 0.0, "y": 0.0, "z": 0.0, "tracking": False}

def send_command(command):
    print(f"Command: {command}")
    if command.startswith("rc"):
        parts = command.split()
        if len(parts) == 5:
            drone.send_rc_control(
                int(parts[1]), int(parts[2]),
                int(parts[3]), int(parts[4]))
    elif command == "takeoff":
        drone.takeoff()
    elif command == "land":
        try:
            drone.land()
        except Exception as e:
            print(f"Land error: {e}")
    elif command == "emergency":
        drone.emergency()

autonomous = AutonomousSearch(
    send_command_func=send_command,
    get_detections_func=get_latest_detections,
    get_position_func=get_position,
    get_point_cloud_func=slam_client.get_point_cloud if USE_SLAM else None
)

# ── VIDEO AND YOLO LOOP ───────────────────────────────────
def video_loop():
    global latest_frame
    print("Video loop started")
    while True:
        frame = frame_reader.frame
        if frame is None:
            continue

        # Send to SLAM
        if USE_SLAM:
            slam_client.send_frame(frame)
            position = slam_client.get_position()
            if position["tracking"]:
                print(f"SLAM pos: {position['x']:.2f}, "
                      f"{position['y']:.2f}, "
                      f"{position['z']:.2f}")

        # Run YOLO
        results = model(frame, verbose=False)
        annotated = results[0].plot()

        # Store frame for main thread display
        with frame_lock:
            latest_frame = annotated

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
            detection_sock.sendto(message, (UNITY_IP, DETECTION_PORT))
            drone_controller_sock.sendto(message, ("127.0.0.1", DETECTION_LISTEN_PORT))
            with detection_lock:
                global latest_detections
                latest_detections = detections

threading.Thread(target=video_loop, daemon=True).start()
# ─────────────────────────────────────────────────────────

# ── POINT CLOUD MONITOR ───────────────────────────────────
def monitor_loop():
    print("Point cloud monitor active")
    last_warning = 0

    while True:
        time.sleep(0.2)

        if not USE_SLAM:
            continue

        point_cloud = slam_client.get_point_cloud()
        drone_pos = slam_client.get_position()

        if not drone_pos["tracking"]:
            continue

        drone_x = drone_pos["x"]
        drone_z = drone_pos["z"]

        forward_points = []
        for point in point_cloud:
            dx = point["x"] - drone_x
            dz = point["z"] - drone_z
            distance = math.sqrt(dx**2 + dz**2)
            if distance < 0.8:
                forward_points.append(point)

        now = time.time()
        if len(forward_points) >= 5 and now - last_warning > 2.0:
            print(f"OBSTACLE: {len(forward_points)} points within 0.8m")
            last_warning = now
        elif len(forward_points) > 0:
            print(f"Points nearby: {len(forward_points)} within 0.8m")

threading.Thread(target=monitor_loop, daemon=True).start()
# ─────────────────────────────────────────────────────────

# ── CONTROLLER LOOP ───────────────────────────────────────
def run():
    print("Controller running")
    print("B: manual  A double tap: autonomous")
    print("Start: takeoff  Back: land  RB: emergency")

    while controller.running:

        # Show YOLO frame on main thread
        with frame_lock:
            frame = latest_frame
        if frame is not None:
            cv2.imshow("Tello YOLO", frame)
        cv2.waitKey(1)

        action = controller.check_buttons()

        if action == "takeoff":
            drone.takeoff()
            time.sleep(2)

        elif action == "land":
            autonomous.stop()
            try:
                drone.land()
            except Exception as e:
                print(f"Land error: {e}")
            time.sleep(2)

        elif action == "emergency":
            autonomous.stop()
            drone.emergency()
            break

        if controller.get_mode() == ControlMode.MANUAL:
            if autonomous.running:
                autonomous.stop()
            lr, fb, ud, yaw = controller.get_rc_values()
            drone.send_rc_control(lr, fb, ud, yaw)

        elif controller.get_mode() == ControlMode.AUTONOMOUS:
            if not autonomous.running:
                autonomous.start()

        elif controller.get_mode() == ControlMode.GAZE:
            if autonomous.running:
                autonomous.stop()
            drone.send_rc_control(0, 0, 0, 0)

        time.sleep(0.05)

try:
    run()
except KeyboardInterrupt:
    autonomous.stop()
    drone.send_rc_control(0, 0, 0, 0)
    try:
        drone.land()
    except Exception:
        pass
finally:
    controller.stop()
    try:
        drone.streamoff()
    except Exception:
        pass
    cv2.destroyAllWindows()
    print("Done")