import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 9998))
sock.settimeout(5.0)

print("Listening on port 9998 for anything...")

while True:
    try:
        data, addr = sock.recvfrom(4096)
        print(f"Got data from {addr}: {data.decode()}")
    except socket.timeout:
        print("Nothing received on 9998")