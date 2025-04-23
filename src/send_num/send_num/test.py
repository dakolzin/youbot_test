import socket
import json

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 5000))  # слушаем все интерфейсы на порту 5000
    server_socket.listen(1)
    print("Сервер запущен и слушает порт 5000...")

    client_socket, addr = server_socket.accept()
    print(f"Клиент подключился: {addr}")

    while True:
        data = client_socket.recv(4096)  # читаем данные порциями по 4K
        if not data:
            break
        msg = json.loads(data.decode('utf-8'))
        print("Получено:", msg)

    print("Клиент отключился, сервер завершает работу.")
    client_socket.close()
    server_socket.close()

if __name__ == '__main__':
    main()
