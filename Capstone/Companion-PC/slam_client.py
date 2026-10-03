import socket
import struct
import threading
import platform
import cv2
import numpy as np

def get_slam_host():
    if platform.system() == "Windows":
        try:
            import subprocess
            result = subprocess.run(
                ["wsl", "-d", "Ubuntu-20.04", "hostname", "-I"],
                capture_output=True, text=True, timeout=5
            )
            return result.stdout.strip().split()[0]
        except Exception:
            return "127.0.0.1"
    else:
        return "127.0.0.1"

SLAM_HOST = get_slam_host()
SLAM_INPUT_PORT = 9100
SLAM_OUTPUT_PORT = 9101

input_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
output_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

current_position = {"x": 0.0, "y": 0.0, "z": 0.0, "tracking": False}
current_points = []
position_lock = threading.Lock()
points_lock = threading.Lock()

def connect():
    print(f"Connecting to {SLAM_HOST}:{SLAM_INPUT_PORT}")
    input_sock.connect((SLAM_HOST, SLAM_INPUT_PORT))
    print("Input connected, waiting for output port...")
    import time
    time.sleep(2)
    print(f"Connecting to {SLAM_HOST}:{SLAM_OUTPUT_PORT}")
    output_sock.connect((SLAM_HOST, SLAM_OUTPUT_PORT))
    print("Connected to SLAM")

def send_frame(frame):
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, jpeg = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, 80])
        data = jpeg.tobytes()
        size = struct.pack("!I", len(data))
        input_sock.sendall(size + data)
    except Exception as e:
        print(f"Frame send error: {e}")

def receive_loop():
    print("Receive loop started")
    while True:
        try:
            # Read position line
            pos_buf = b""
            while b"\n" not in pos_buf:
                chunk = output_sock.recv(4096)
                if not chunk:
                    return
                pos_buf += chunk

            pos_line, remainder = pos_buf.split(b"\n", 1)
            pos_line = pos_line.decode().strip()

            parts = pos_line.split(",")
            if len(parts) == 4:
                try:
                    tracking = parts[3].strip() == "1"
                    with position_lock:
                        current_position["x"] = float(parts[0])
                        current_position["y"] = float(parts[1])
                        current_position["z"] = float(parts[2])
                        current_position["tracking"] = tracking

                    # Clear point cloud when not tracking
                    if not tracking:
                        with points_lock:
                            current_points.clear()

                except ValueError:
                    pass

            # Read exactly 4 bytes for point cloud size
            size_buf = remainder
            while len(size_buf) < 4:
                chunk = output_sock.recv(4 - len(size_buf))
                if not chunk:
                    return
                size_buf += chunk

            points_size = struct.unpack("I", size_buf[:4])[0]
            extra = size_buf[4:]

            if points_size == 0:
                continue

            # Read point cloud data
            points_data = extra
            while len(points_data) < points_size:
                chunk = output_sock.recv(
                    min(65536, points_size - len(points_data)))
                if not chunk:
                    return
                points_data += chunk

            points_str = points_data[:points_size].decode(
                "utf-8", errors="ignore")

            points = []
            for point_str in points_str.split(";"):
                point_str = point_str.strip()
                if not point_str:
                    continue
                coords = point_str.split(",")
                if len(coords) == 3:
                    try:
                        points.append({
                            "x": float(coords[0]),
                            "y": float(coords[1]),
                            "z": float(coords[2])
                        })
                    except ValueError:
                        pass

            with points_lock:
                current_points.clear()
                current_points.extend(points)

        except Exception as e:
            print(f"Receive error: {e}")
            break

def get_position():
    with position_lock:
        return current_position.copy()

def get_point_cloud():
    with points_lock:
        return list(current_points)

def start():
    print("Connecting to SLAM...")
    connect()
    thread = threading.Thread(target=receive_loop, daemon=True)
    thread.start()
    print("SLAM client running, receive thread started")