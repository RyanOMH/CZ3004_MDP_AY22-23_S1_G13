import socket


class Client:
    """
    Used as the client for RPI.
    """
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = socket.socket()
    
    def connect(self):
        print(f"Attempting connection to ALGO at {self.host}:{self.port}")
        self.socket.connect((self.host, self.port))
        print("Connected to ALGO!")

    def receive(self):
        msg = self.socket.recv(1024)
        data = msg.decode()
        print(data)

    def send(self, data):
        msg = data.encode()
        self.socket.send(msg)
    
    def close(self):
        print("Closing client socket.")
        self.socket.close()


if __name__ == '__main__':
    client = Client(socket.gethostbyname(socket.gethostname()), 3004)
    client.connect()

    client.receive()
    print("Move forward")
    client.receive()
    print("Scan")
    client.send("null\n")
    print("Image not detected, continue moving forward")
    client.receive()
    print("Move forward")
    client.send("Aknowledgement from STM")
    client.receive()

