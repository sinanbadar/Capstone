import socket
import time

UNITY_IP = "127.0.0.1"
COMMAND_PORT = 8889
RESPONSE_PORT = 8890

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

send_command("takeoff")
time.sleep(4)

send_command("forward 50")
time.sleep(4)

send_command("left 30")
time.sleep(4)

send_command("cw 90")
time.sleep(4)

send_command("land")
time.sleep(3)

sock.close()
print("Done")

# cd C:\Users\Sinan\Capstone\Companion-PC
# .\venv\Scripts\activate