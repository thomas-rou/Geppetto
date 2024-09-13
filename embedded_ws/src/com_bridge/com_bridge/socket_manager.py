# com_bridge/socket_manager.py

import socket

class SocketManager:
    def __init__(self):
        self.client_socket = None
        self.server_ip = None
        self.port = None

    def connect(self, server_ip, port):
        if self.client_socket:
            self.client_socket.close()
        self.server_ip = server_ip
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.server_ip, self.port))

    def send(self, data):
        if self.client_socket:
            self.client_socket.sendall(data.encode('utf-8'))

    def close(self):
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
