import socket
import pickle
HEADERSIZE = 10


class Server:
    """
    Used as the server for ALGO.
    """
    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = host 
        self.port = port

    def start(self):
        print(f"Creating server at {self.host}:{self.port}")
        self.socket.bind((self.host, self.port))
        print(f"Socket binded to port {self.port}")
        self.socket.listen()
        print("Listening for connection...")
        self.clientSocket, self.address = self.socket.accept()
        print('Got connection from', self.address)

    def send(self, d):
        msg = pickle.dumps(d)
        self.clientSocket.send(msg)
    
    def receive(self):
        msg = self.clientSocket.recv(1024)
        d = pickle.loads(msg)
        return(d)
        
    def close(self):
        print("Closing server socket.")
        self.socket.close()

if __name__ == '__main__':
    server = Server("10.27.196.78", 3004)
    server.start()
    server.receive()