import socket
import time
from bullseye_server import Server
from bullseyealgo_minimal import AlgoMinimal
from bullseye_parse import parse_obstacle_data


def main():
    complete = 0

    # Create a client to send and receive information from the RPi
    #client = Client("192.168.13.1", 3004)
    #client.connect()
    server = Server(socket.gethostbyname(socket.gethostname()), 3004)
    # For testing, use client.py

    server.start()
    while True:
        try:
            #server.send("AND|HELLO")
            pass
        except KeyboardInterrupt:
            server.close()
    """
    obstacle = [[105, 105, -90, 0]]
    obs = parse_obstacle_data(obstacle)
    print(f"Obstacle position {obs}")
    bullseye_app = AlgoMinimal(obs)
    pos = bullseye_app.robot.get_current_pos()
    print(f"Starting robot position {pos.get_xy()}")

    retry = True
    while retry:
        try:
            server.send("AND|READY TO START\n")
            # server.receive()
            print("Begin moving forward")
            server.send("STM|FC005\n")
            for i in range(5):
                pos.increase_y()
                print(f"Robot position {pos.get_xy()}")
                i*=1
            server.send(f"AND|{pos.get_xy()}\n")
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
                    for i in range(10):
                        pos.decrease_y()
                        print(f"Robot position {pos.get_xy()}")
                        i+=1
                    server.send(f"AND|{pos.get_xy()}\n")
                    server.receive()
                    server.send("STM|FL090\n")
                    server.receive()
                    server.send("STM|FR090\n")
                    server.receive()
                    server.send("STM|FC020\n")
                    for i in range(20):
                        pos.increase_y()
                        print(f"Robot position {pos.get_xy()}")
                        i+=1
                    server.send(f"AND|{pos.get_xy()}\n")
                    server.receive()
                    server.send("STM|BL090\n")
                    server.receive()
                    server.send("STM|FR010\n")
                    server.receive()
                    server.send("STM|FC030\n")
                    server.receive()
                elif image_id == "null\n" or image_id == "null":
                    server.send("STM|FC005\n")
                    for i in range(5):
                        pos.increase_y()
                        print(f"AND|pos.get_xy()")
                        i+=1
                    server.send(f"AND|{pos.get_xy()}\n")
                    server.receive()
                else:
                    # If image is finally detected
                    complete = 1
                    string_to_android = f"AND_IMAGE|Obstacle1,{image_id}"
                    server.send(string_to_android)
                    retry = False
                    break

        except KeyboardInterrupt:
        """
            # server.close()
            # break
        



if __name__ == '__main__':
    main()