#!/usr/bin/env python3
# coding: utf-8
"""
Мини-сервер на localhost:5000, который принимает JSON+'\n' и выводит его в консоль.
"""

import socket
import threading
import json

HOST = "127.0.0.1"
PORT = 5000
RECV_BUFFER = 4096

def handle_client(conn, addr):
    print(f"[+] Клиент подключился: {addr}")
    buf = b""
    try:
        while True:
            data = conn.recv(RECV_BUFFER)
            if not data:
                print(f"[-] Клиент {addr} отключился")
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    pkt = json.loads(line.decode())
                except json.JSONDecodeError:
                    print("[!] Плохой JSON, пропускаем:", line)
                    continue
                print(f"[RECV] {pkt}")
    finally:
        conn.close()

def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"[*] Тест-сервер слушает {HOST}:{PORT}")
    try:
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        print("\n[!] Остановка сервера")
    finally:
        srv.close()

if __name__ == "__main__":
    main()
