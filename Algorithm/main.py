import json
import time
from turtle import distance
from Simulator.simulator import AlgoSimulator, AlgoMinimal
from Simulator.simulator_mgr import parse_obstacle_data_cur
from Connection.client import Client
from Navigate.roam import roam


def main(simulator):
    """ simulator: Pass in True to show simulator screen
    """
    index = 0
    i = 0
    reverse = "STM|BC020"
    scan = "RPI|"
    forward = "STM|FC020"
    reverseSecond = "STM|BC010"
    forwardSecond = "STM|FC030"
    obst_list = []
    image_ids = ["11", "12", "13", "14", "15", "16", "17", "18", "19", "20", 
                 "21", "22", "23", "24", "25", "26", "27", "28", "29", "30",
                 "30", "31", "32", "33", "34", "35", "36", "37", "38", "39", "40"]

    # Create a client to send and receive information from the RPi
    client = Client("10.27.131.94", 3004)  # 10.27.146 139 | 192.168.13.1
    client.connect()

    while True:
        try:
            # ANDROID send obstacle positions to ALGO
            print("===========================Receive Obstacles Data===========================")
            print("Waiting to receive obstacle data from ANDROID...")
            obstacle_data = client.receive()
            data2 = json.loads(obstacle_data)
            obst_list.append(data2)

            while obstacle_data != "PC;START":
                obstacle_data = client.receive()
                if obstacle_data == "PC;START":
                    break
                data = json.loads(obstacle_data)
                print(data)
                obst_list.append(data)
                i+=1
            
            obst_list.pop()
            print("Received all obstacles data from ANDROID.")
            print(f"Obstacles data: {obst_list}")

            print("============================Parse Obstacles Data============================")
            obstacles = parse_obstacle_data_cur(obst_list)
            print(f"After parsing: {obstacles}")

            print("===============================Calculate path===============================")
            if simulator == True:
                app = AlgoSimulator(obstacles)
                app.init()
                app.execute()
            else:
                app = AlgoMinimal(obstacles)
                index_list = app.execute()
            commands = app.robot.convert_commands()
            print("Full list of paths commands till last obstacle:")
            print(f"{commands}")
            roboPosCoor = {
                "x": 15,
                "y": 4,
                "direction" : "N"
            }
            
            print("=======================Send path commands to move to obstacles=======================")
            for command in commands:
                print(f"Sending path commands to move to obstacle {index_list[index]} to RPI to STM...")
                print(command)
                client.send(command)
                if(command != "RPI|"):
                    updateRoboPos(roboPosCoor, command)
                    roboUpdateToAndroid = f"AND|ROBOT,<{roboPosCoor['x']//10}>,<{roboPosCoor['y']//10}>,<{roboPosCoor['direction']}>"
                    client.send(roboUpdateToAndroid)

    
                print("Waiting to receive aknowledgement/image_id from STM/IMAGE REC")
                var = client.receive()
                print(var)
                if var == "CMPLT":
                    continue
                elif var == "n":
                    print("No image detected. Sending command to reverse...")
                    client.send(reverse)
                    updateRoboPos(roboPosCoor, reverse)
                    roboUpdateToAndroid = f"AND|ROBOT,<{roboPosCoor['x']//10}>,<{roboPosCoor['y']//10}>,<{roboPosCoor['direction']}>"
                    client.send(roboUpdateToAndroid)
                    print("Waiting to receive aknowledgement for reverse")
                    client.receive()
                    client.send(scan)
                    print("Waiting to receive image_id from IMAGE REC")
                    var = client.receive()
                    if var.strip() in image_ids:
                        print(f"Received image id. Sending image id to ANDROID")
                        var2 = var.strip()
                        string_to_android = f"AND_IMAGE|TARGET,<{index_list[index]+1}>,<{var2}>"
                        client.send(string_to_android)
                        time.sleep(0.5)
                        index += 1
                        print("Moving robot forward to original position...")
                        client.send(forward)
                        updateRoboPos(roboPosCoor, forward)
                        roboUpdateToAndroid = f"AND|ROBOT,<{roboPosCoor['x']//10}>,<{roboPosCoor['y']//10}>,<{roboPosCoor['direction']}>"
                        client.send(roboUpdateToAndroid)
                        print("Waiting to receive aknowledgement")
                        client.receive()
                    elif var == "n":
                        print("No image detected. Sending command to reverse more...")
                        client.send(reverseSecond)
                        updateRoboPos(roboPosCoor, reverseSecond)
                        roboUpdateToAndroid = f"AND|ROBOT,<{roboPosCoor['x']//10}>,<{roboPosCoor['y']//10}>,<{roboPosCoor['direction']}>"
                        client.send(roboUpdateToAndroid)
                        print("Waiting to receive aknowledgement for reverse")
                        client.receive()
                        client.send(scan)
                        print("Waiting to receive image_id from IMAGE REC")
                        var = client.receive()
                        if var.strip() in image_ids:
                            print(f"Received image id. Sending image id to ANDROID")
                            var2 = var.strip()
                            string_to_android = f"AND_IMAGE|TARGET,<{index_list[index]+1}>,<{var2}>"
                            client.send(string_to_android)
                            time.sleep(0.5)
                            index += 1
                            print("Moving robot forward to original position...")
                            client.send(forwardSecond)
                            updateRoboPos(roboPosCoor, forwardSecond)
                            roboUpdateToAndroid = f"AND|ROBOT,<{roboPosCoor['x']//10}>,<{roboPosCoor['y']//10}>,<{roboPosCoor['direction']}>"
                            client.send(roboUpdateToAndroid)
                            print("Waiting to receive aknowledgement")
                            client.receive()
                elif var.strip() in image_ids:
                    print(f"Received image id. Sending image id to ANDROID")
                    var2 = var.strip()
                    string_to_android = f"AND_IMAGE|TARGET,<{index_list[index]+1}>,<{var2}>"
                    client.send(string_to_android)
                    time.sleep(0.5)
                    index += 1
                else:
                    break
                
        except KeyboardInterrupt:
            client.close()

def updateRoboPos(roboPos,command):
    stmCommand = command[4:]
    print(f"Remote update of Robot: {stmCommand}")
    if(stmCommand[0:2] == "FC"):
        stmDist = int(stmCommand[2:])
        if(roboPos["direction"] == "N"):
            roboPos["y"] = roboPos["y"] + stmDist
        elif(roboPos["direction"] == "S"):
            roboPos["y"] = roboPos["y"] - stmDist
        elif(roboPos["direction"] == "E"):
            roboPos["x"] = roboPos["x"] + stmDist
        else:
            roboPos["x"] = roboPos["x"] - stmDist
    elif(stmCommand[0:2] == "BC"):
        stmDist = int(stmCommand[2:])
        if(roboPos["direction"] == "N"):
            roboPos["y"] = roboPos["y"] - stmDist
        elif(roboPos["direction"] == "S"):
            roboPos["y"] = roboPos["y"] + stmDist
        elif(roboPos["direction"] == "E"):
            roboPos["x"] = roboPos["x"] - stmDist
        else:
            roboPos["x"] = roboPos["x"] + stmDist
    elif(stmCommand[0:2] == "FR"):
        stmDist = 30
        if(roboPos["direction"] == "N"):
            roboPos["x"] = roboPos["x"] + stmDist
            roboPos["y"] = roboPos["y"] + stmDist
            roboPos["direction"] = "E"
        elif(roboPos["direction"] == "S"):
            roboPos["x"] = roboPos["x"] - stmDist
            roboPos["y"] = roboPos["y"] - stmDist
            roboPos["direction"] = "W"
        elif(roboPos["direction"] == "E"):
            roboPos["x"] = roboPos["x"] + stmDist
            roboPos["y"] = roboPos["y"] - stmDist
            roboPos["direction"] = "S"
        else:
            roboPos["x"] = roboPos["x"] - stmDist
            roboPos["y"] = roboPos["y"] + stmDist
            roboPos["direction"] = "N"
    elif(stmCommand[0:2] == "FL"):
        stmDist = 30
        if(roboPos["direction"] == "N"):
            roboPos["x"] = roboPos["x"] - stmDist
            roboPos["y"] = roboPos["y"] + stmDist
            roboPos["direction"] = "W"
        elif(roboPos["direction"] == "S"):
            roboPos["x"] = roboPos["x"] + stmDist
            roboPos["y"] = roboPos["y"] - stmDist
            roboPos["direction"] = "E"
        elif(roboPos["direction"] == "E"):
            roboPos["x"] = roboPos["x"] + stmDist
            roboPos["y"] = roboPos["y"] + stmDist
            roboPos["direction"] = "N"
        else:
            roboPos["x"] = roboPos["x"] - stmDist
            roboPos["y"] = roboPos["y"] - stmDist
            roboPos["direction"] = "S"
    elif(stmCommand[0:2] == "BR"):
        stmDist = 30
        if(roboPos["direction"] == "N"):
            roboPos["x"] = roboPos["x"] + stmDist
            roboPos["y"] = roboPos["y"] - stmDist
            roboPos["direction"] = "W"
        elif(roboPos["direction"] == "S"):
            roboPos["x"] = roboPos["x"] - stmDist
            roboPos["y"] = roboPos["y"] + stmDist
            roboPos["direction"] = "E"
        elif(roboPos["direction"] == "E"):
            roboPos["x"] = roboPos["x"] - stmDist
            roboPos["y"] = roboPos["y"] - stmDist
            roboPos["direction"] = "N"
        else:
            roboPos["x"] = roboPos["x"] + stmDist
            roboPos["y"] = roboPos["y"] + stmDist
            roboPos["direction"] = "S"
    elif(stmCommand[0:2] == "BL"):
        stmDist = 30
        if(roboPos["direction"] == "N"):
            roboPos["x"] = roboPos["x"] - stmDist
            roboPos["y"] = roboPos["y"] - stmDist
            roboPos["direction"] = "E"
        elif(roboPos["direction"] == "S"):
            roboPos["x"] = roboPos["x"] + stmDist
            roboPos["y"] = roboPos["y"] + stmDist
            roboPos["direction"] = "W"
        elif(roboPos["direction"] == "E"):
            roboPos["x"] = roboPos["x"] - stmDist
            roboPos["y"] = roboPos["y"] + stmDist
            roboPos["direction"] = "S"
        else:
            roboPos["x"] = roboPos["x"] + stmDist
            roboPos["y"] = roboPos["y"] - stmDist
            roboPos["direction"] = "N"
    else:
        pass

    if(roboPos["x"]//10 >= 19):
        roboPos["x"] = 180
    elif(roboPos["x"]//10 <= 0):
        roboPos["x"] = 10
    elif(roboPos["y"]//10 >= 19):
        roboPos["y"] = 180
    elif(roboPos["y"]//10 <= 0):
        roboPos["y"] = 10
    else:
        pass


if __name__ == '__main__':
    main(False)