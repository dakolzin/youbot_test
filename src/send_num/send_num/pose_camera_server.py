#!/usr/bin/env python3
import socket, time, itertools

# Список положений и ориентаций
POSE_LINES = [
    "0.342 -0.297 0.047 0.797 -0.081 0.244 0.547\n",
    "-0.150 -0.384 -0.267 -0.178 0.210 0.848 0.453\n",
    "-0.172 -0.389 -0.181 0.164 0.466 0.817 0.298\n",
    "0.389 -0.102 0.471 -0.359 -0.350 -0.604 0.620\n",
    "-0.035 -0.028 0.285 -0.460 -0.098 0.284 0.835\n",
]

HOST, PORT = "127.0.0.1", 6000

def main() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"pose_server: listening on {HOST}:{PORT}")

        # Бесконечный цикл, перебирающий список по кругу
        for pose in itertools.cycle(POSE_LINES):
            conn, addr = s.accept()
            with conn:
                print(f"connection from {addr}, sending pose: {pose.strip()}")
                conn.sendall(pose.encode())
            time.sleep(0.1)      # минимальная пауза, чтобы не грузить CPU

if __name__ == "__main__":
    main()
