import socket
import time
import slam_client  # position now comes from SLAM, not from Unity telemetry

# ── Real Tello addresses ─────────────────────────────────
DRONE_IP = "192.168.10.1"
COMMAND_PORT = 8889
LOCAL_COMMAND_PORT = 8889   # Tello replies to whatever port you sent from
# ─────────────────────────────────────────────────────────

drone_state = {
    "pos_x": 0.0,
    "pos_y": 0.0,
    "pos_z": 0.0,
    "tracking": False
}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", LOCAL_COMMAND_PORT))
sock.settimeout(5.0)


def connect():
    sock.sendto(b"command", (DRONE_IP, COMMAND_PORT))
    try:
        resp, _ = sock.recvfrom(1024)
        print(f"Connected: {resp.decode()}")
        return True
    except socket.timeout:
        print("No response, check WiFi connection to Tello")
        return False


def update_position():
    if not USE_SLAM:
        return
    position = slam_client.get_position()
    drone_state["tracking"] = position["tracking"]
    if position["tracking"]:
        drone_state["pos_x"] = position["x"]
        drone_state["pos_y"] = position["y"]
        drone_state["pos_z"] = position["z"]


def send_command(command):
    print(f"Sending: {command}")
    sock.sendto(command.encode(), (DRONE_IP, COMMAND_PORT))
    try:
        response, _ = sock.recvfrom(1024)
        print(f"Response: {response.decode()}")
    except socket.timeout:
        print("No response received")

    update_position()
    status = "tracking" if drone_state["tracking"] else "lost"
    print(f"Position ({status}): {drone_state['pos_x']:.2f}, "
          f"{drone_state['pos_y']:.2f}, {drone_state['pos_z']:.2f}")


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

if not connect():
    sock.close()
    raise SystemExit("Could not connect to Tello, aborting")

send_command("streamon")
time.sleep(1)

send_command("takeoff")
time.sleep(5)

send_command("forward 50")
time.sleep(5)

send_command("left 30")
time.sleep(5)

send_command("cw 90")
time.sleep(5)

send_command("land")
time.sleep(3)

send_command("streamoff")
sock.close()
print("Done")

# cd C:\Users\Sinan\Capstone\Capstone\Companion-PC
# .\venv\Scripts\activate