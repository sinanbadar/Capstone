import socket
import time
import json
import telemetry_receiver  # type: ignore

drone_state = {
    "pos_x": 0.0,
    "pos_y": 0.0,
    "pos_z": 0.0,
    "rot_x": 0.0,
    "rot_y": 0.0,
    "rot_z": 0.0,
    "is_flying": False
}

def send_command(command):
    print(f"Sending: {command}")
    sock.sendto(command.encode(), (UNITY_IP, COMMAND_PORT))
    try:
        response, _ = sock.recvfrom(4096)
        response_str = response.decode()
        try:
            state = json.loads(response_str)
            drone_state.update(state)
            print(f"Position: {state['pos_x']:.2f}, {state['pos_y']:.2f}, {state['pos_z']:.2f}")
        except json.JSONDecodeError:
            print(f"Response: {response_str}")
    except socket.timeout:
        print("No response received")

telemetry_receiver.start()

UNITY_IP = "127.0.0.1"
COMMAND_PORT = 8889
RESPONSE_PORT = 8891

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", RESPONSE_PORT))
sock.settimeout(3.0)

def send_command(command):
    print(f"Sending: {command}")
    sock.sendto(command.encode(), (UNITY_IP, COMMAND_PORT))
    try:
        response, _ = sock.recvfrom(1024)
        print(f"Response: {response.decode()}")
    except socket.timeout:
        print("No response received")

send_command("command")
time.sleep(1)
state = telemetry_receiver.get_drone_state()
print(f"Drone position: {state['pos_x']:.2f}, {state['pos_y']:.2f}, {state['pos_z']:.2f}")

send_command("takeoff")
time.sleep(5)
state = telemetry_receiver.get_drone_state()
print(f"Drone position: {state['pos_x']:.2f}, {state['pos_y']:.2f}, {state['pos_z']:.2f}")

send_command("forward 50")
time.sleep(5)
state = telemetry_receiver.get_drone_state()
print(f"Drone position: {state['pos_x']:.2f}, {state['pos_y']:.2f}, {state['pos_z']:.2f}")

send_command("left 30")
time.sleep(5)
state = telemetry_receiver.get_drone_state()
print(f"Drone position: {state['pos_x']:.2f}, {state['pos_y']:.2f}, {state['pos_z']:.2f}")

send_command("cw 90")
time.sleep(5)
state = telemetry_receiver.get_drone_state()
print(f"Drone position: {state['pos_x']:.2f}, {state['pos_y']:.2f}, {state['pos_z']:.2f}")

send_command("land")
time.sleep(3)
state = telemetry_receiver.get_drone_state()
print(f"Drone position: {state['pos_x']:.2f}, {state['pos_y']:.2f}, {state['pos_z']:.2f}")

sock.close()
print("Done")

# cd C:\Users\Sinan\Capstone\Capstone\Companion-PC
# .\venv\Scripts\activate