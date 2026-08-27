import socket
import struct
import cv2
import numpy as np
import threading

SLAM_HOST = "127.0.0.1"
SLAM_INPUT_PORT = 9100
SLAM_OUTPUT_PORT = 9101

slam_position = {"x": 0.0, "y": 0.0, "z": 0.0, "tracking": False}
position_lock = threading.Lock()

input_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
output_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def connect():
    print("Connecting to SLAM...")
    input_sock.connect((SLAM_HOST, SLAM_INPUT_PORT))
    output_sock.connect((SLAM_HOST, SLAM_OUTPUT_PORT))
    print("Connected to SLAM")

def send_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, jpeg = cv2.imencode('.jpg', gray, [cv2.IMWRITE_JPEG_QUALITY, 90])
    data = jpeg.tobytes()
    size = struct.pack('<I', len(data))
    input_sock.sendall(size + data)

def receive_position():
    while True:
        try:
            data = output_sock.recv(256).decode()
            if data:
                parts = data.strip().split(',')
                if len(parts) == 4:
                    with position_lock:
                        slam_position["x"] = float(parts[0])
                        slam_position["y"] = float(parts[1])
                        slam_position["z"] = float(parts[2])
                        slam_position["tracking"] = parts[3] == "1"
        except:
            break

def get_position():
    with position_lock:
        return slam_position.copy()

def start():
    connect()
    thread = threading.Thread(target=receive_position, daemon=True)
    thread.start()
    print("SLAM client running")