import socket
import pickle


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

    def send(self, d):
        msg = d.encode()
        self.socket.send(msg)

    def receive(self):
        msg = self.socket.recv(1024)
        data = msg.decode()
    
    def close(self):
        print("Closing client socket.")
        self.socket.close()


if __name__ == '__main__':
    client = Client(socket.gethostbyname(socket.gethostname()), 3004)
    client.connect()
    # x 18 y 3 -> x 185 y 35
    obstacles = [[15, 185, -90, 0], [65, 125, 90, 1], [105, 75, 0, 2], [155, 165, 180, 3], [185, 95, 180, 4], [135, 25, 0, 5]]
    client.send(obstacles)
    client.receive()
    client.receive()
    client.send('success')
    client.receive()
    client.receive()
    client.send('success')
    client.receive()
    client.receive()
    client.send('success')
    client.receive()
    client.receive()
    client.send('success')
    client.receive()
    client.receive()
    client.send('success')
    client.receive()
    client.receive()
    client.send('success')
    """
    for i in range(len(obstacles)):
        client.receive()
        client.send('success')
    """
