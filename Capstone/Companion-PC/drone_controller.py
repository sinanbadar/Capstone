import socket
import time
import telemetry_receiver  # type: ignore
from controller_input import ControllerInput, ControlMode

# ── MODE SWITCH ──────────────────────────────────────────
USE_REAL_TELLO = False
# ─────────────────────────────────────────────────────────

UNITY_IP = "127.0.0.1"
COMMAND_PORT = 8889
RESPONSE_PORT = 8893

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

def send_command(command):
    sock.sendto(command.encode(), (UNITY_IP, COMMAND_PORT))
    try:
        response, _ = sock.recvfrom(1024)
        print(f"Response: {response.decode()}")
    except socket.timeout:
        pass
    except ConnectionResetError:
        print("Unity not in Play mode")

def run():
    print("Controller running")
    print("Start=takeoff  Back=land  RB=emergency stop")
    print("Left stick: forward back left right")
    print("Right stick: up down yaw")
    print("A double tap: autonomous  B: manual  X: gaze")

    send_command("command")
    time.sleep(0.5)

    while controller.running:
        action = controller.check_buttons()

        if action == "takeoff":
            if USE_REAL_TELLO:
                drone.takeoff()
            else:
                send_command("takeoff")
            time.sleep(2)

        elif action == "land":
            if USE_REAL_TELLO:
                drone.land()
            else:
                send_command("land")
            time.sleep(2)

        elif action == "emergency":
            if USE_REAL_TELLO:
                drone.emergency()
            else:
                send_command("emergency")
            break

        if controller.get_mode() == ControlMode.MANUAL:
            lr, fb, ud, yaw = controller.get_rc_values()

            if USE_REAL_TELLO:
                # ── REAL TELLO ────────────────────────────
                drone.send_rc_control(lr, fb, ud, yaw)
                # ─────────────────────────────────────────
            else:
                # ── SIMULATION ───────────────────────────
                rc_command = f"rc {lr} {fb} {ud} {yaw}"
                send_command(rc_command)
                # ─────────────────────────────────────────

        elif controller.get_mode() == ControlMode.AUTONOMOUS:
            if USE_REAL_TELLO:
                drone.send_rc_control(0, 0, 0, 0)
            else:
                send_command("rc 0 0 0 0")

        time.sleep(0.05)

try:
    run()
except KeyboardInterrupt:
    if USE_REAL_TELLO:
        drone.send_rc_control(0, 0, 0, 0)
        drone.land()
    else:
        send_command("land")
finally:
    sock.close()
    controller.stop()
    print("Done")