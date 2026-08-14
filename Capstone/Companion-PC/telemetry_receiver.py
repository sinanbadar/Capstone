import socket
import json
import threading

TELEMETRY_PORT = 9998

drone_state = {
    "pos_x": 0.0,
    "pos_y": 0.0,
    "pos_z": 0.0,
    "rot_x": 0.0,
    "rot_y": 0.0,
    "rot_z": 0.0,
    "is_flying": False,
    "timestamp": 0.0
}

state_lock = threading.Lock()

def start_telemetry_receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", TELEMETRY_PORT))
    sock.settimeout(1.0)
    print(f"Telemetry receiver listening on port {TELEMETRY_PORT}")

    while True:
        try:
            data, addr = sock.recvfrom(4096)
            json_str = data.decode("utf-8")
            state = json.loads(json_str)
            with state_lock:
                drone_state.update(state)
            print(f"Telemetry: pos({state['pos_x']:.2f}, {state['pos_y']:.2f}, {state['pos_z']:.2f}) flying:{state['is_flying']}")
        except socket.timeout:
            pass
        except Exception as e:
            print(f"Telemetry error: {e}")

def get_drone_state():
    with state_lock:
        return drone_state.copy()

def start():
    thread = threading.Thread(target=start_telemetry_receiver, daemon=True)
    thread.start()
    return thread