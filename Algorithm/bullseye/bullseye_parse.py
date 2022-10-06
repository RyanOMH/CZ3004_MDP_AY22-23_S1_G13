from typing import List
from bullseye_map.bullseye_obstacle import Obstacle
from bullseye_settings import *


def parse_obstacle_data(data) -> List[Obstacle]:
    obs = []

    for obstacle_params in data:
        obs.append(Obstacle(obstacle_params[0],
                            obstacle_params[1],
                            Direction(obstacle_params[2]),
                            obstacle_params[3]))
        # [[x, y, orient, index], [x, y, orient, index]]
        return obs