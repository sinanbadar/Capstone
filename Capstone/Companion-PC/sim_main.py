import cv2
import json
import math
import numpy as np
import socket
import time
import threading
from ultralytics import YOLO
from controller_input import ControllerInput, ControlMode
from autonomous_search import AutonomousSearch

from depth_estimator import DepthEstimator
depth_est = DepthEstimator(threshold=0.3)
depth_obstacle = False
depth_lock = threading.Lock()

latest_depth_frame = None
depth_frame_lock = threading.Lock()

def get_depth_obstacle():
    with depth_lock:
        return depth_obstacle

# ── CONFIG ───────────────────────────────────────────────
UNITY_IP = "127.0.0.1"
COMMAND_PORT = 8889
RESPONSE_PORT = 8893
VIDEO_PORT = 11111
DETECTION_PORT = 9999
DETECTION_LISTEN_PORT = 9996
OPTICAL_FLOW_THRESHOLD = 8.0
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

# ── SOCKETS ──────────────────────────────────────────────
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
cmd_sock.bind(("", RESPONSE_PORT))
cmd_sock.settimeout(1.0)

video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
video_sock.bind(("0.0.0.0", VIDEO_PORT))
video_sock.settimeout(3.0)

detection_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone_controller_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# ─────────────────────────────────────────────────────────

import telemetry_receiver  # type: ignore
telemetry_receiver.start()

model = YOLO("yolov8n.pt")
controller = ControllerInput(unity_ip=UNITY_IP)

# ── SHARED STATE ─────────────────────────────────────────
latest_detections = []
detection_lock = threading.Lock()
latest_frame = None
frame_lock = threading.Lock()
optical_flow_obstacle = False
flow_lock = threading.Lock()
prev_gray_frame = None

def get_latest_detections():
    with detection_lock:
        return latest_detections.copy()

def get_optical_flow_obstacle():
    with flow_lock:
        return optical_flow_obstacle

def get_position():
    if USE_SLAM:
        return slam_client.get_position()
    return telemetry_receiver.get_drone_state()
# ─────────────────────────────────────────────────────────

def check_optical_flow(frame):
    global prev_gray_frame, optical_flow_obstacle

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

    prev_gray_frame = curr_gray

    if mean_mag > OPTICAL_FLOW_THRESHOLD:
        print(f"Optical flow obstacle: {mean_mag:.2f}")
        with flow_lock:
            optical_flow_obstacle = True
        return True

    with flow_lock:
        optical_flow_obstacle = False
    return False

def send_command(command):
    print(f"Command: {command}")
    cmd_sock.sendto(command.encode(), (UNITY_IP, COMMAND_PORT))
    try:
        response, _ = cmd_sock.recvfrom(1024)
        print(f"Response: {response.decode()}")
    except socket.timeout:
        pass
    except ConnectionResetError:
        print("Unity not in Play mode")

autonomous = AutonomousSearch(
    get_depth_func=get_depth_obstacle,
    send_command_func=send_command,
    get_detections_func=get_latest_detections,
    get_position_func=get_position,
    get_point_cloud_func=slam_client.get_point_cloud if USE_SLAM else None,
    get_optical_flow_func=get_optical_flow_obstacle
)

# ── VIDEO AND YOLO LOOP ───────────────────────────────────
def video_loop():
    global latest_frame
    print("Video loop started")
    while True:
        try:
            data, addr = video_sock.recvfrom(65536)
            np_array = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

            if frame is None:
                continue

            # Depth estimation
            depth_map, depth_vis = depth_est.estimate(frame)
            with depth_lock:
                global depth_obstacle
                depth_obstacle = depth_est.obstacle_ahead(depth_map)
            with depth_frame_lock:
                global latest_depth_frame
                latest_depth_frame = depth_vis

            check_optical_flow(frame)

            if USE_SLAM:
                slam_client.send_frame(frame)
                position = slam_client.get_position()
                if position["tracking"]:
                    print(f"SLAM pos: {position['x']:.2f}, "
                          f"{position['y']:.2f}, "
                          f"{position['z']:.2f}")

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

        except socket.timeout:
            pass
        except Exception as e:
            print(f"Video error: {e}")

threading.Thread(target=video_loop, daemon=True).start()
# ─────────────────────────────────────────────────────────

# ── CONTROLLER LOOP ───────────────────────────────────────
def run():
    print("Simulation running")
    print("B: manual  A double tap: autonomous")
    print("Start: takeoff  Back: land  RB: emergency")

    send_command("command")
    time.sleep(0.5)

    while controller.running:

        with frame_lock:
            frame = latest_frame
        if frame is not None:
            cv2.imshow("Sim YOLO", frame)

        with depth_frame_lock:
            dframe = latest_depth_frame
        if dframe is not None:
            cv2.imshow("Depth", dframe) 
            
        cv2.waitKey(1)

        action = controller.check_buttons()

        if action == "takeoff":
            send_command("takeoff")
            time.sleep(2)

        elif action == "land":
            autonomous.stop()
            send_command("land")
            time.sleep(2)

        elif action == "emergency":
            autonomous.stop()
            send_command("emergency")
            break

        if controller.get_mode() == ControlMode.MANUAL:
            if autonomous.running:
                autonomous.stop()
            lr, fb, ud, yaw = controller.get_rc_values()
            send_command(f"rc {lr} {fb} {ud} {yaw}")

        elif controller.get_mode() == ControlMode.AUTONOMOUS:
            if not autonomous.running:
                autonomous.start()

        elif controller.get_mode() == ControlMode.GAZE:
            if autonomous.running:
                autonomous.stop()

        time.sleep(0.05)

try:
    run()
except KeyboardInterrupt:
    autonomous.stop()
    send_command("land")
finally:
    cmd_sock.close()
    video_sock.close()
    detection_sock.close()
    controller.stop()
    cv2.destroyAllWindows()
    print("Done")