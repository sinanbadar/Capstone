import faulthandler
faulthandler.enable()

import os
os.environ["TRANSFORMERS_OFFLINE"] = "1"

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
import autonomous_search as asr
from autonomous_search import AutonomousSearch
from depth_estimator import DepthEstimator
from occupancy_map import OccupancyMap

logging.getLogger('djitellopy').setLevel(logging.WARNING)

# CONFIG
DRONE_IP = "192.168.10.1"
#UNITY_IP = ""
UNITY_IP = "127.0.0.1"          # Unity editor on this Mac. Use the headset IP for a build
DETECTION_PORT = 9999
DETECTION_LISTEN_PORT = 9996
ALERT_PORT = 9995
MAP_PORT = 9994
TELEMETRY_PORT = 8891
VIDEO_STREAM_PORT = 9093

TARGET_CLASS = "person"
TARGET_CONFIDENCE = 0.5

# REAL DRONE TUNING (recalibrate on the real camera, the sim values do not carry over)
DEPTH_THRESHOLD = 5.5
OPEN_DEPTH_THRESHOLD = 4.2
OPTICAL_FLOW_THRESHOLD = 20.0
REAL_FORWARD_SPEED = 40          # start low, raise once stopping is reliable
REAL_YAW_SPEED = 70
REAL_YAW_DEG_PER_SEC = 30.0      # time a full turn on the real drone and set this
REAL_EST_SPEED_MPS = 0.30        # measure: fly 3s at FORWARD_SPEED over a taped distance

asr.FORWARD_SPEED = REAL_FORWARD_SPEED
asr.YAW_SPEED = REAL_YAW_SPEED
asr.YAW_DEG_PER_SEC = REAL_YAW_DEG_PER_SEC
asr.EST_SPEED_MPS = REAL_EST_SPEED_MPS
asr.OPEN_DEPTH_THRESHOLD = OPEN_DEPTH_THRESHOLD
asr.STOP_DEPTH_THRESHOLD = DEPTH_THRESHOLD

# SLAM
USE_SLAM = True
if USE_SLAM:
    import slam_client
    try:
        slam_client.start()
        print("SLAM client connected")
    except Exception as e:
        print(f"SLAM failed: {e}")
        USE_SLAM = False

# DRONE
drone = Tello(host=DRONE_IP)
drone.connect()
print(f"Battery: {drone.get_battery()}%")
drone.streamon()
frame_reader = drone.get_frame_read()

# SOCKETS, MODELS, CONTROLLER
alert_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
map_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_stream_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
detection_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone_controller_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

model = YOLO("yolov8n.pt")
controller = ControllerInput(unity_ip=UNITY_IP)
room_map = OccupancyMap()

# SHARED STATE
latest_detections = []
detection_lock = threading.Lock()
latest_frame = None
frame_lock = threading.Lock()

optical_flow_obstacle = False
flow_lock = threading.Lock()
prev_gray_frame = None
pause_optical_flow = False
flow_pause_lock = threading.Lock()
flow_trigger_count = 0
flow_frame_count = 0

depth_est = DepthEstimator(threshold=DEPTH_THRESHOLD)
depth_obstacle = False
depth_lock = threading.Lock()
latest_depth_frame = None
depth_frame_lock = threading.Lock()
latest_frame_for_depth = None
depth_input_lock = threading.Lock()
latest_depth_map = None
latest_depth_time = 0.0
depth_map_lock = threading.Lock()

last_target_log = 0.0


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


def get_depth_obstacle():
    with depth_lock:
        return depth_obstacle


def get_latest_depth_map():
    with depth_map_lock:
        return latest_depth_map


def get_centre_depth():
    with depth_map_lock:
        if latest_depth_map is None:
            return 99.0
        return depth_est.get_centre_depth(latest_depth_map)


def get_fresh_depth(after_time, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        with depth_map_lock:
            if latest_depth_map is not None and latest_depth_time > after_time:
                return depth_est.get_centre_depth(latest_depth_map)
        time.sleep(0.05)
    return 99.0   # no fresh data counts as blocked


def reset_depth():
    global depth_obstacle
    with depth_lock:
        depth_obstacle = False
        depth_est.trigger_count = 0
        if hasattr(depth_est, "history"):
            depth_est.history = []


def get_position():
    if USE_SLAM:
        return slam_client.get_position()
    return {"x": 0.0, "y": 0.0, "z": 0.0, "tracking": False}


def send_alert(active, message=""):
    try:
        alert_sock.sendto(json.dumps({"active": active, "message": message}).encode(),
                          (UNITY_IP, ALERT_PORT))
    except OSError:
        pass


def map_update(x, z, heading):
    dm = get_latest_depth_map()
    if dm is not None:
        room_map.update(dm, x, z, heading)


def stream_frame_to_unity(frame):
    try:
        small = cv2.resize(frame, (320, 240))
        _, jpeg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
        data = jpeg.tobytes()
        chunk_size = 60000
        chunks = [data[i:i + chunk_size] for i in range(0, len(data), chunk_size)]
        for i, chunk in enumerate(chunks):
            header = f"{i}:{len(chunks)}:".encode()
            video_stream_sock.sendto(header + chunk, (UNITY_IP, VIDEO_STREAM_PORT))
    except Exception:
        pass


def check_optical_flow(frame):
    global prev_gray_frame, optical_flow_obstacle
    global flow_trigger_count, flow_frame_count

    with flow_pause_lock:
        paused = pause_optical_flow
    if paused:
        with flow_lock:
            optical_flow_obstacle = False
        prev_gray_frame = None
        flow_trigger_count = 0
        return

    curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if prev_gray_frame is None:
        prev_gray_frame = curr_gray
        return

    flow = cv2.calcOpticalFlowFarneback(
        prev_gray_frame, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    h, w = curr_gray.shape
    centre = flow[h // 4:3 * h // 4, w // 4:3 * w // 4]
    mean_mag = float(np.mean(np.sqrt(centre[..., 0] ** 2 + centre[..., 1] ** 2)))
    prev_gray_frame = curr_gray

    flow_frame_count += 1
    if flow_frame_count % 60 == 0:
        print(f"FLOW: {mean_mag:.2f} thr {OPTICAL_FLOW_THRESHOLD}")

    if mean_mag > OPTICAL_FLOW_THRESHOLD:
        flow_trigger_count += 1
        if flow_trigger_count >= 3:
            with flow_lock:
                optical_flow_obstacle = True
            return
    else:
        flow_trigger_count = 0
        with flow_lock:
            optical_flow_obstacle = False


def send_command(command):
    if command.startswith("rc"):
        parts = command.split()
        if len(parts) == 5:
            drone.send_rc_control(int(parts[1]), int(parts[2]),
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
    pause_flow_func=set_flow_pause,
    get_depth_func=get_depth_obstacle,
    get_depth_map_func=get_latest_depth_map,
    get_centre_depth_func=get_centre_depth,
    get_fresh_depth_func=get_fresh_depth,
    reset_depth_func=reset_depth,
    alert_func=send_alert,
    map_update_func=map_update,
    room_map=room_map,
)


# BACKGROUND SENDERS (started after everything they use exists)
def telemetry_loop():
    while True:
        try:
            s = drone.get_current_state()
            pos = get_position()
            state = {
                "pos_x": float(pos.get("x", 0.0)),
                "pos_y": float(pos.get("y", 0.0)),
                "pos_z": float(pos.get("z", 0.0)),
                "is_flying": bool(drone.is_flying),
                "battery": int(s.get("bat", 0)),
                "height_m": float(s.get("h", 0)) / 100.0,
                "speed_mps": float((s.get("vgx", 0) ** 2 + s.get("vgy", 0) ** 2) ** 0.5) / 100.0,
            }
            telemetry_sock.sendto(json.dumps(state).encode(), (UNITY_IP, TELEMETRY_PORT))
        except Exception as e:
            print(f"TELEMETRY: {e}")
        time.sleep(0.5)


def map_sender():
    while True:
        try:
            walls = [v for cell in room_map.export() for v in cell]   # [x0, z0, x1, z1, ...]
            msg = {"walls": walls,
                   "drone_x": autonomous.est_x,
                   "drone_z": autonomous.est_z,
                   "heading": autonomous.current_heading,
                   **room_map.stats()}
            map_sock.sendto(json.dumps(msg).encode(), (UNITY_IP, MAP_PORT))
        except OSError:
            pass
        time.sleep(1.0)


def depth_loop():
    global depth_obstacle, latest_depth_frame, latest_depth_map, latest_depth_time
    while True:
        with depth_input_lock:
            frame = latest_frame_for_depth
        if frame is None:
            time.sleep(0.05)
            continue
        try:
            depth_map, depth_vis = depth_est.estimate(frame)
            with depth_map_lock:
                latest_depth_map = depth_map
                latest_depth_time = time.time()
            with depth_lock:
                depth_obstacle = depth_est.obstacle_ahead(depth_map)
            with depth_frame_lock:
                latest_depth_frame = depth_vis
        except Exception as e:
            print(f"Depth error: {e}")
            time.sleep(0.1)


def video_loop():
    global latest_frame, latest_frame_for_depth, latest_detections, last_target_log
    frame_count = 0
    last_beat = time.time()
    while True:
        try:
            frame = frame_reader.frame
            if frame is None:
                time.sleep(0.01)
                continue

            frame_count += 1
            if time.time() - last_beat > 5.0:
                print(f"VIDEO: {frame_count} frames")
                last_beat = time.time()

            stream_frame_to_unity(frame)

            with depth_input_lock:
                latest_frame_for_depth = frame.copy()

            check_optical_flow(frame)

            if USE_SLAM:
                slam_client.send_frame(frame)

            results = model(frame, verbose=False)
            with frame_lock:
                latest_frame = results[0].plot()

            detections = []
            for box in results[0].boxes:
                conf = float(box.conf[0])
                label = model.names[int(box.cls[0])]
                xyxy = box.xyxy[0].tolist()

                if label == TARGET_CLASS and conf >= TARGET_CONFIDENCE:
                    now = time.time()
                    if now - last_target_log > 3.0:
                        pos = get_position()
                        print(f"TARGET: {label} {conf:.2f} "
                              f"pos({pos.get('x', 0):.2f},{pos.get('z', 0):.2f})")
                        last_target_log = now

                detections.append({
                    "label": label,
                    "confidence": round(conf, 2),
                    "bbox": {"x1": round(xyxy[0]), "y1": round(xyxy[1]),
                             "x2": round(xyxy[2]), "y2": round(xyxy[3])},
                    "slam_pos": slam_client.get_position() if USE_SLAM else None,
                })

            if detections:
                message = json.dumps(detections).encode()
                try:
                    detection_sock.sendto(message, (UNITY_IP, DETECTION_PORT))
                except OSError:
                    pass
                try:
                    drone_controller_sock.sendto(message, ("127.0.0.1", DETECTION_LISTEN_PORT))
                except OSError:
                    pass
                with detection_lock:
                    latest_detections = detections

        except Exception as e:
            print(f"VIDEO ERROR: {e}")
            time.sleep(0.1)


threading.Thread(target=telemetry_loop, daemon=True).start()
threading.Thread(target=map_sender, daemon=True).start()
threading.Thread(target=depth_loop, daemon=True).start()
threading.Thread(target=video_loop, daemon=True).start()


def do_takeoff():
    try:
        drone.takeoff()
    except Exception as e:
        print(f"Takeoff error: {e}")


# CONTROLLER LOOP
def run():
    print("Ready: B=manual  A(x2)=autonomous  Start=takeoff  Back=land  RB=emergency")

    while controller.running:
        with frame_lock:
            frame = latest_frame
        if frame is not None:
            cv2.imshow("Tello YOLO", frame)

        with depth_frame_lock:
            dframe = latest_depth_frame
        if dframe is not None:
            cv2.imshow("Depth", dframe)

        cv2.waitKey(1)

        action = controller.check_buttons()

        if action == "takeoff":
            print("Takeoff requested")
            threading.Thread(target=do_takeoff, daemon=True).start()

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

        mode = controller.get_mode()
        if mode == ControlMode.MANUAL:
            if autonomous.running:
                autonomous.stop()
            lr, fb, ud, yaw = controller.get_rc_values()
            scale = 0.4
            drone.send_rc_control(int(lr * scale), int(fb * scale),
                                  int(ud * scale), int(yaw * scale))

        elif mode == ControlMode.AUTONOMOUS:
            if not autonomous.running:
                autonomous.start()

        elif mode == ControlMode.GAZE:
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