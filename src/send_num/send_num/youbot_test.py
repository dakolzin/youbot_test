#!/usr/bin/env python3
import socket, time

# Параметры точки захвата (можно менять на лету)
POSE_LINE = "0.433988 0.160393 0.043095 0.928814 0.370542 -0.001494 0.000243\n"
HOST, PORT = "0.0.0.0", 5000

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    print(f"pose_server: listening on {HOST}:{PORT}")
    while True:
        conn, addr = s.accept()
        with conn:
            print(f"connection from {addr}")
            conn.sendall(POSE_LINE.encode())
        time.sleep(0.1)
