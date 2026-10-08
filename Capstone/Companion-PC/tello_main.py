import cv2
import json
import math
import numpy as np
import socket
import time
import threading
import logging
from djitellopy import Tello
from ultralytics import YOLO
from controller_input import ControllerInput, ControlMode
from autonomous_search import AutonomousSearch

logging.getLogger('djitellopy').setLevel(logging.WARNING)

# ── CONFIG ───────────────────────────────────────────────
DRONE_IP = "192.168.10.1"
UNITY_IP = "127.0.0.1"
DETECTION_PORT = 9999
DETECTION_LISTEN_PORT = 9996
OPTICAL_FLOW_THRESHOLD = 20.0
# ─────────────────────────────────────────────────────────

# ── SLAM ─────────────────────────────────────────────────
USE_SLAM = True
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

# ── SHARED STATE ─────────────────────────────────────────
latest_detections = []
detection_lock = threading.Lock()
latest_frame = None
frame_lock = threading.Lock()
optical_flow_obstacle = False
flow_lock = threading.Lock()
prev_gray_frame = None
pause_optical_flow = False
flow_pause_lock = threading.Lock()

def get_latest_detections():
    with detection_lock:
        return latest_detections.copy()

def get_optical_flow_obstacle():
    with flow_lock:
        return optical_flow_obstacle

def set_flow_pause(paused):
    global pause_optical_flow
    with flow_pause_lock:
        pause_optical_flow = paused

def get_position():
    if USE_SLAM:
        return slam_client.get_position()
    return {"x": 0.0, "y": 0.0, "z": 0.0, "tracking": False}
# ─────────────────────────────────────────────────────────

def check_optical_flow(frame):
    global prev_gray_frame, optical_flow_obstacle

    with flow_pause_lock:
        if pause_optical_flow:
            with flow_lock:
                optical_flow_obstacle = False
            prev_gray_frame = None
            return False

    curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if prev_gray_frame is None:
        prev_gray_frame = curr_gray
        return False

    flow = cv2.calcOpticalFlowFarneback(
        prev_gray_frame, curr_gray, None,
        0.5, 3, 15, 3, 5, 1.2, 0)

    h, w = curr_gray.shape
    centre = flow[h//4:3*h//4, w//4:3*w//4]
    magnitude = np.sqrt(centre[..., 0]**2 + centre[..., 1]**2)
    mean_mag = float(np.mean(magnitude))

    if not hasattr(check_optical_flow, 'frame_count'):
        check_optical_flow.frame_count = 0
        check_optical_flow.trigger_count = 0
    check_optical_flow.frame_count += 1
    if check_optical_flow.frame_count % 30 == 0:
        print(f"FLOW: magnitude {mean_mag:.2f} threshold {OPTICAL_FLOW_THRESHOLD}")

    prev_gray_frame = curr_gray

    if mean_mag > OPTICAL_FLOW_THRESHOLD:
        check_optical_flow.trigger_count += 1
        if check_optical_flow.trigger_count >= 3:
            print(f"FLOW: OBSTACLE confirmed at {mean_mag:.2f}")
            with flow_lock:
                optical_flow_obstacle = True
            return True
    else:
        check_optical_flow.trigger_count = 0
        with flow_lock:
            optical_flow_obstacle = False
    return False

def send_command(command):
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
    get_point_cloud_func=slam_client.get_point_cloud if USE_SLAM else None,
    get_optical_flow_func=get_optical_flow_obstacle,
    pause_flow_func=set_flow_pause
)

# ── VIDEO AND YOLO LOOP ───────────────────────────────────
def video_loop():
    global latest_frame
    while True:
        frame = frame_reader.frame
        if frame is None:
            continue

        check_optical_flow(frame)

        if USE_SLAM:
            slam_client.send_frame(frame)

        results = model(frame, verbose=False)
        annotated = results[0].plot()

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

        if detections:
            message = json.dumps(detections).encode()
            detection_sock.sendto(message, (UNITY_IP, DETECTION_PORT))
            drone_controller_sock.sendto(
                message, ("127.0.0.1", DETECTION_LISTEN_PORT))
            with detection_lock:
                global latest_detections
                latest_detections = detections

threading.Thread(target=video_loop, daemon=True).start()
# ─────────────────────────────────────────────────────────

# ── POINT CLOUD MONITOR ───────────────────────────────────
def monitor_loop():
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
            print(f"SLAM: {len(forward_points)} points within 0.8m")
            last_warning = now

threading.Thread(target=monitor_loop, daemon=True).start()
# ─────────────────────────────────────────────────────────

# ── CONTROLLER LOOP ───────────────────────────────────────
def run():
    print("Ready: B=manual  A(x2)=autonomous  Start=takeoff  Back=land  RB=emergency")

    while controller.running:

        with frame_lock:
            frame = latest_frame
        if frame is not None:
            cv2.imshow("Tello YOLO", frame)
        cv2.waitKey(1)

        action = controller.check_buttons()

        if action == "takeoff":
            try:
                drone.takeoff()
                time.sleep(2)
            except Exception as e:
                print(f"Takeoff error: {e}")

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