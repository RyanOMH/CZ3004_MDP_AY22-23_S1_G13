import json
from typing import List
from Map.obstacle import Obstacle
from Settings.direction import Direction


def parse_obstacle_data_old(data) -> List[Obstacle]:
    obs = []

    for obstacle_params in data:
        obs.append(Obstacle(obstacle_params[0],
                            obstacle_params[1],
                            Direction(obstacle_params[2]),
                            obstacle_params[3]))
                            
    # [[x, y, orient, index], [x, y, orient, index]]
    return obs

def parse_obstacle_data_cur(data) -> List[Obstacle]:
    obs = []
    lst3 = []
    lst = []
    i = 0

    for obj in data:
        lst.append(obj)

    for i in lst:
        i["x"] *= 10
        i["x"] += 5
        i["y"] *= 10
        i["y"] += 5
        i["obs_id"] -= 1

    a = [list(row) for row in zip(*[m.values() for m in lst])]

    for i in range(len(a[0])):
        lst2 = [item[i] for item in a]
        lst3.append(lst2)
        i+=1
        
    for obstacle_params in lst3:
        obs.append(Obstacle(obstacle_params[0],
                            obstacle_params[1],
                            Direction(obstacle_params[2]),
                            obstacle_params[3]))

    # [[x, y, orient, index], [x, y, orient, index]]
    return obs


def parse_obstacle_data_new(data) -> List[Obstacle]:
    i = 0
    obst_list = []
    obs = []
    d = json.loads(data)
    for i in range(len(d)):
        obst_str = "obstacle"
        obst_str_num = obst_str + str(i+1)
        obst_list.append(d[obst_str_num])
        i+=1

    for obstacle_params in obst_list:
        obs.append(Obstacle(obstacle_params[0],
                            obstacle_params[1],
                            Direction(obstacle_params[2]),
                            obstacle_params[3]))
                            
    # [[x, y, orient, index], [x, y, orient, index]]
    return obs