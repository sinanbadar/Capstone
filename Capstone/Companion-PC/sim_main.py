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
from ultralytics import YOLO
from controller_input import ControllerInput, ControlMode
import autonomous_search as asr
from autonomous_search import AutonomousSearch
from depth_estimator import DepthEstimator
from occupancy_map import OccupancyMap
room_map = OccupancyMap()

def map_update(x, z, heading):
    dm = get_latest_depth_map()
    if dm is not None:
        room_map.update(dm, x, z, heading)

# CONFIG
UNITY_IP = "127.0.0.1"
COMMAND_PORT = 8889
RESPONSE_PORT = 8893
VIDEO_PORT = 11111
DETECTION_PORT = 9999
DETECTION_LISTEN_PORT = 9996

TARGET_CLASS = "person"
ALERT_PORT = 9995
alert_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_alert(active, message=""):
    try:
        alert_sock.sendto(json.dumps({"active": active, "message": message}).encode(),
                          (UNITY_IP, ALERT_PORT))
    except OSError:
        pass
TARGET_CONFIDENCE = 0.5

# Sim tuning. Calibrate these on Unity frames, the scale differs from the real Tello.
DEPTH_THRESHOLD = 3.0          # stop threshold, read p90 from the depth window overlay
OPEN_DEPTH_THRESHOLD = 4.2     # a heading at or below this counts as open
FLOW_THRESHOLD = 8.0
SIM_FORWARD_SPEED = 30
SIM_YAW_SPEED = 50
SIM_YAW_DEG_PER_SEC = SIM_YAW_SPEED * 1.2     # = 60.0 with the values above     # measure in Unity: degrees turned per second at SIM_YAW_SPEED

asr.FORWARD_SPEED = SIM_FORWARD_SPEED
asr.YAW_SPEED = SIM_YAW_SPEED
asr.YAW_DEG_PER_SEC = SIM_YAW_DEG_PER_SEC
asr.OPEN_DEPTH_THRESHOLD = OPEN_DEPTH_THRESHOLD

# SLAM
USE_SLAM = False   # set True to use the slam_client.py module, which connects to the SLAM server in Unity
if USE_SLAM:
    import slam_client
    try:
        slam_client.start()
        print("SLAM client connected")
    except Exception as e:
        print(f"SLAM failed: {e}")
        USE_SLAM = False

# SOCKETS
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
cmd_sock.bind(("", RESPONSE_PORT))
cmd_sock.settimeout(1.0)
cmd_lock = threading.Lock()

video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
video_sock.bind(("0.0.0.0", VIDEO_PORT))
video_sock.settimeout(3.0)

detection_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone_controller_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

import telemetry_receiver  # type: ignore
telemetry_receiver.start()

model = YOLO("yolov8n.pt")
controller = ControllerInput(unity_ip=UNITY_IP)

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
    state = telemetry_receiver.get_drone_state()
    return {"x": state.get("pos_x", 0.0), "y": state.get("pos_y", 0.0),
            "z": state.get("pos_z", 0.0), "tracking": False}


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
        print(f"FLOW: {mean_mag:.2f} thr {FLOW_THRESHOLD}")

    if mean_mag > FLOW_THRESHOLD:
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
    try:
        if command.startswith("rc"):
            # rc commands do not wait for a reply, so autonomous movement stays smooth
            cmd_sock.sendto(command.encode(), (UNITY_IP, COMMAND_PORT))
            return
        with cmd_lock:
            print(f"Command: {command}")
            cmd_sock.sendto(command.encode(), (UNITY_IP, COMMAND_PORT))
            try:
                response, _ = cmd_sock.recvfrom(1024)
                print(f"Response: {response.decode()}")
            except socket.timeout:
                pass
    except ConnectionResetError:
        print("Unity not in Play mode")
    except OSError as e:
        print(f"Command send error: {e}")


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

map_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
MAP_PORT = 9994

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

threading.Thread(target=map_sender, daemon=True).start()

# DEPTH LOOP
def depth_loop():
    global depth_obstacle, latest_depth_frame, latest_depth_map, latest_depth_time
    frame_count = 0
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

            frame_count += 1
            if frame_count % 30 == 0:
                with raw_lock:
                    age = time.time() - raw_time
                print(f"DEPTH LAG: {age:.2f}s")
        except Exception as e:
            print(f"Depth error: {e}")
            time.sleep(0.1)
        



# VIDEO AND YOLO LOOP
# VIDEO: READER + PROCESSOR
latest_raw = None
raw_time = 0.0
raw_lock = threading.Lock()


def frame_reader_loop():
    global latest_raw, raw_time
    while True:
        try:
            data, _ = video_sock.recvfrom(65536)
            frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            with raw_lock:
                latest_raw = frame
                raw_time = time.time()
            if USE_SLAM:
                slam_client.send_frame(frame)
        except socket.timeout:
            pass
        except Exception as e:
            print(f"READER ERROR: {e}")
            time.sleep(0.1)


def video_loop():
    global latest_frame, latest_frame_for_depth, latest_detections, last_target_log
    last_seen = 0.0
    print("Video loop started")
    while True:
        try:
            with raw_lock:
                frame = latest_raw
                t = raw_time
            if frame is None or t == last_seen:
                time.sleep(0.01)
                continue
            last_seen = t

            with depth_input_lock:
                latest_frame_for_depth = frame.copy()

            check_optical_flow(frame)

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
                        print(f"TARGET: {label} {conf:.2f} pos({pos.get('x', 0):.2f},{pos.get('z', 0):.2f})")
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


threading.Thread(target=frame_reader_loop, daemon=True).start()
threading.Thread(target=video_loop, daemon=True).start()
threading.Thread(target=depth_loop, daemon=True).start()


# CONTROLLER LOOP
def run():
    print("Simulation running")
    print("B: manual  A(x2): autonomous  Start: takeoff  Back: land  RB: emergency")

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

        mode = controller.get_mode()
        if mode != getattr(run, "last_mode", None):
            print(f"MODE: {mode}")
            run.last_mode = mode
        if mode == ControlMode.MANUAL:
            if autonomous.running:
                autonomous.stop()
            lr, fb, ud, yaw = controller.get_rc_values()
            #print(f"RC from controller: {lr} {fb} {ud} {yaw}")
            send_command(f"rc {lr} {fb} {ud} {yaw}")

        elif mode == ControlMode.AUTONOMOUS:
            if not autonomous.running:
                autonomous.start()

        time.sleep(0.05)
TRIAL_MODE = True   # True = automated trials, False = manual control
ENV_NAME = "plain_bright"           # change per environment
START_POSE = (0.0, 0.0, 0.0)        # x, z, yaw for this environment
RUN_SECONDS = 30
REPEATS = 2
CONDITIONS = {"depth_only": (True, False), "flow_only": (False, True), "depth_flow": (True, True)}

stop_counter = [0]

def on_stop(info):
    stop_counter[0] += 1
    st = telemetry_receiver.get_drone_state()
    print(f"*** STOP #{stop_counter[0]}  reason={info.get('reason')}  "
          f"pos=({st['pos_x']:.2f}, {st['pos_z']:.2f})  yaw={st['rot_y']:.0f}")

autonomous.on_stop = on_stop


def run_trials():
    asr.USE_YOLO_OBSTACLE = False
    asr.USE_SLAM_OBSTACLE = False

    while get_latest_depth_map() is None:
        time.sleep(0.5)
    send_command("takeoff")
    time.sleep(3)

    for cond, (use_d, use_f) in CONDITIONS.items():
        asr.USE_DEPTH, asr.USE_FLOW = use_d, use_f
        for rep in range(1, REPEATS + 1):
            send_command(f"reset {START_POSE[0]} {START_POSE[1]} {START_POSE[2]}")
            time.sleep(0.5)
            set_flow_pause(True)
            time.sleep(0.6)
            get_fresh_depth(time.time() + 0.3)
            reset_depth()
            set_flow_pause(False)

            stop_counter[0] = 0
            print(f"\n===== {ENV_NAME} | {cond} | run {rep} =====")
            controller.set_mode(ControlMode.AUTONOMOUS)
            time.sleep(RUN_SECONDS)
            controller.set_mode(ControlMode.MANUAL)
            time.sleep(4)

    print("TRIALS DONE")


if TRIAL_MODE:
    threading.Thread(target=run_trials, daemon=True).start()

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