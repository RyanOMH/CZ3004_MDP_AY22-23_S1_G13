import socket


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
        msg = d.encode()
        self.clientSocket.send(msg)
    
    def receive(self):
        msg = self.clientSocket.recv(1024)
        data = msg.decode()
        return(data)
        
    def close(self):
        print("Closing server socket.")
        self.socket.close()


def bullseye():
    complete = 0

    # Create a server to send and receive information from the RPi
    server = Server(socket.gethostbyname(socket.gethostname()), 3004)
    # For testing, use client.py
    server.start()

    while True:
        try:
            print("Begin moving forward")
            server.send("STM|FC005\n")
            server.receive()

            while complete == 0:
                print("Scan")
                server.send("RPI|")
                image_id = str(server.receive())
                print(f"Current image id is {image_id}")

                if image_id == "4\n" or image_id == "4":
                    print("Bullseye detected...")
                    print("Moving robot to left face of obstacle")
                    server.send("STM|BC010\n")
                    server.receive()
                    server.send("STM|FL090\n")
                    server.receive()
                    server.send("STM|FR090\n")
                    server.receive()
                    server.send("STM|FC020\n")
                    server.receive()
                    server.send("STM|BL090\n")
                    server.receive()
                    server.send("STM|FR010\n")
                    server.receive()
                    server.send("STM|FC030\n")
                    server.receive()
                elif image_id == "null\n" or image_id == "null":
                    server.send("STM|FC005\n")
                    server.receive()
                else:
                    # If image is finally detected
                    complete = 1
                    string_to_android = f"AND_IMAGE|Obstacle1,{image_id}"
                    server.send(string_to_android)
                    break

        except KeyboardInterrupt:
            server.close()
            break
          



if __name__ == '__main__':
    bullseye()