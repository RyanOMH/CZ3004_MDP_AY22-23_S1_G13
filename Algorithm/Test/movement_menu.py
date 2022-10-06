import socket
from server import Server


def movement_menu():
    # Create a server to send and receive information from the RPi
    server = Server(socket.gethostbyname(socket.gethostname()), 3004)
    server.start()
    # For testing, use client.py

    print("Waiting to receive movement command from RPI...")
    # ANDROID send movement command to RPI to ALGO
    movement = server.receive()
    print("Received movement command from RPI.")

    while movement != "end":
        if movement == "forward":
            server.send(["FC10"])
        elif movement == "backward":
            server.send(["BC10"])
    
    server.close()


if __name__ == '__main__':
    movement_menu()