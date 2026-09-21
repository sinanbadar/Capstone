import socket
import time
import threading
import json
import telemetry_receiver  # type: ignore
from controller_input import ControllerInput, ControlMode
from autonomous_search import AutonomousSearch

# ── MODE SWITCH ──────────────────────────────────────────
USE_REAL_TELLO = False
# ─────────────────────────────────────────────────────────

UNITY_IP = "127.0.0.1"
COMMAND_PORT = 8889
RESPONSE_PORT = 8893
DETECTION_LISTEN_PORT = 9996

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", RESPONSE_PORT))
sock.settimeout(1.0)

telemetry_receiver.start()

if USE_REAL_TELLO:
    from djitellopy import Tello
    drone = Tello(host="192.168.0.7")
    drone.connect()
    print(f"Battery: {drone.get_battery()}%")

controller = ControllerInput(unity_ip=UNITY_IP)

# ── DETECTION SHARING ─────────────────────────────────────
latest_detections = []
detection_lock = threading.Lock()

def detection_listener():
    det_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    det_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    det_sock.bind(("0.0.0.0", DETECTION_LISTEN_PORT))
    det_sock.settimeout(1.0)
    print(f"Detection listener on port {DETECTION_LISTEN_PORT}")
    while True:
        try:
            data, _ = det_sock.recvfrom(65536)
            detections = json.loads(data.decode())
            with detection_lock:
                global latest_detections
                latest_detections = detections
        except socket.timeout:
            pass
        except Exception as e:
            print(f"Detection error: {e}")

threading.Thread(target=detection_listener, daemon=True).start()

def get_latest_detections():
    with detection_lock:
        return latest_detections.copy()
# ─────────────────────────────────────────────────────────

def send_command(command):
    print(f"Sending: {command}")
    sock.sendto(command.encode(), (UNITY_IP, COMMAND_PORT))
    try:
        response, _ = sock.recvfrom(1024)
        print(f"Response: {response.decode()}")
    except socket.timeout:
        pass
    except ConnectionResetError:
        print("Unity not in Play mode")

autonomous = AutonomousSearch(
    send_command_func=send_command,
    get_detections_func=get_latest_detections,
    get_position_func=telemetry_receiver.get_drone_state
)

def run():
    print("Controller running")
    print("B: manual  A double tap: autonomous  X: gaze")
    print("Start: takeoff  Back: land  RB: emergency")

    send_command("command")
    time.sleep(0.5)

    print("Starting in autonomous mode")
    autonomous.start()

    while controller.running:
        action = controller.check_buttons()

        if action == "takeoff":
            if USE_REAL_TELLO:
                drone.takeoff()
            else:
                send_command("takeoff")
            time.sleep(2)

        elif action == "land":
            autonomous.stop()
            if USE_REAL_TELLO:
                drone.land()
            else:
                send_command("land")
            time.sleep(2)

        elif action == "emergency":
            autonomous.stop()
            if USE_REAL_TELLO:
                drone.emergency()
            else:
                send_command("emergency")
            break

        if controller.get_mode() == ControlMode.MANUAL:
            if autonomous.running:
                autonomous.stop()

            lr, fb, ud, yaw = controller.get_rc_values()

            if USE_REAL_TELLO:
                drone.send_rc_control(lr, fb, ud, yaw)
            else:
                rc_command = f"rc {lr} {fb} {ud} {yaw}"
                send_command(rc_command)

        elif controller.get_mode() == ControlMode.AUTONOMOUS:
            if not autonomous.running:
                autonomous.start()
            if USE_REAL_TELLO:
                drone.send_rc_control(0, 0, 0, 0)

        elif controller.get_mode() == ControlMode.GAZE:
            if autonomous.running:
                autonomous.stop()

        time.sleep(0.05)

try:
    run()
except KeyboardInterrupt:
    autonomous.stop()
    if USE_REAL_TELLO:
        drone.send_rc_control(0, 0, 0, 0)
        drone.land()
    else:
        send_command("land")
finally:
    sock.close()
    controller.stop()
    print("Done")