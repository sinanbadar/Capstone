import socket
import json
import threading
import time

USE_SLAM = False  # flip back on once cv2/slam_client are sorted
if USE_SLAM:
    import slam_client

STATUS_PUSH_RATE = 0.1  # seconds, matches old TelloSimulator telemetryRate

# ── Real Tello ────────────────────────────────────────────
DRONE_IP = "192.168.10.1"
COMMAND_PORT = 8889
STATE_PORT = 8890      # Tello broadcasts its own telemetry here, unprompted
VIDEO_PORT = 11111
# ─────────────────────────────────────────────────────────

# ── Unity ─────────────────────────────────────────────────
UNITY_IP = "127.0.0.1"
UNITY_COMMAND_PORT = 8889   # Unity sends commands here (matches your sim script)
UNITY_RESPONSE_PORT = 8891  # Unity listens for state JSON here
# ─────────────────────────────────────────────────────────

drone_state = {
    "pos_x": 0.0,
    "pos_y": 0.0,
    "pos_z": 0.0,
    "is_flying": False,
    "battery": 0,
    "height_m": 0.0,
    "speed_mps": 0.0,
}

# Socket that talks to the real Tello (command channel)
tello_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_sock.bind(("", COMMAND_PORT))
tello_sock.settimeout(5.0)

# Socket that receives Tello's unsolicited state broadcasts
state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock.bind(("", STATE_PORT))

# Socket that talks to Unity
unity_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
unity_sock.bind(("", UNITY_COMMAND_PORT + 100))  # avoid clashing with tello_sock locally


def parse_tello_state(raw: str) -> dict:
    """Tello state string looks like: 'pitch:0;roll:0;...;h:120;bat:87;...'"""
    fields = {}
    for pair in raw.strip().strip(";").split(";"):
        if ":" not in pair:
            continue
        key, val = pair.split(":", 1)
        try:
            fields[key] = float(val)
        except ValueError:
            fields[key] = val
    return fields


def state_listener():
    while True:
        try:
            data, _ = state_sock.recvfrom(1024)
            fields = parse_tello_state(data.decode())

            drone_state["battery"] = int(fields.get("bat", 0))
            tof_cm = fields.get("tof", 0)
            drone_state["height_m"] = fields.get("h", 0) / 100.0

            vgx = fields.get("vgx", 0)
            vgy = fields.get("vgy", 0)
            vgz = fields.get("vgz", 0)
            speed_cm_s = (vgx ** 2 + vgy ** 2 + vgz ** 2) ** 0.5
            drone_state["speed_mps"] = speed_cm_s / 100.0
        except Exception as e:
            print(f"State parse error: {e}")


def update_slam_position():
    if not USE_SLAM:
        return  # pos_x/y/z stay at last known value (0 until SLAM is back)
    position = slam_client.get_position()
    if position.get("tracking"):
        drone_state["pos_x"] = position["x"]
        drone_state["pos_y"] = position["y"]
        drone_state["pos_z"] = position["z"]


def connect():
    tello_sock.sendto(b"command", (DRONE_IP, COMMAND_PORT))
    try:
        resp, _ = tello_sock.recvfrom(1024)
        print(f"Connected: {resp.decode()}")
        return True
    except socket.timeout:
        print("No response, check WiFi connection to Tello")
        return False


def send_to_tello(cmd: str) -> str:
    tello_sock.sendto(cmd.encode(), (DRONE_IP, COMMAND_PORT))
    try:
        resp, _ = tello_sock.recvfrom(1024)
        return resp.decode()
    except socket.timeout:
        return "no_response"


def push_status_to_unity():
    payload = json.dumps(drone_state).encode()
    unity_sock.sendto(payload, (UNITY_IP, UNITY_RESPONSE_PORT))


def status_push_loop():
    """Sends status to Unity continuously, independent of commands,
    so the panel updates even while the drone is just sitting there
    (battery drain, SLAM drift, etc) and not only right after a command."""
    while True:
        update_slam_position()
        push_status_to_unity()
        time.sleep(STATUS_PUSH_RATE)


def command_relay_loop():
    while True:
        cmd_bytes, _ = unity_sock.recvfrom(1024)
        cmd = cmd_bytes.decode()
        print(f"Unity -> Tello: {cmd}")

        reply = send_to_tello(cmd)
        print(f"Tello -> Unity: {reply}")

        if cmd == "takeoff":
            drone_state["is_flying"] = True
        elif cmd == "land":
            drone_state["is_flying"] = False

        # status_push_loop already keeps Unity updated continuously,
        # but push immediately here too so the ack feels responsive
        update_slam_position()
        push_status_to_unity()


if __name__ == "__main__":
    if USE_SLAM:
        try:
            slam_client.start()
            print("SLAM client connected")
        except Exception as e:
            print(f"SLAM connection failed: {e}, positions will stay at 0")
    else:
        print("SLAM disabled, position will stay at 0,0,0")

    if not connect():
        raise SystemExit("Could not connect to Tello, aborting")

    threading.Thread(target=state_listener, daemon=True).start()
    threading.Thread(target=status_push_loop, daemon=True).start()

    print("Bridge running. Unity -> send commands to "
          f"{UNITY_IP}:{UNITY_COMMAND_PORT + 100}, "
          f"listen for state on {UNITY_RESPONSE_PORT}")

    command_relay_loop()