from typing import List
from abc import ABC, abstractmethod
from bullseye_map.bullseye_obstacle import Obstacle
from bullseye_map.bullseye_grid import Grid
from bullseye_robot.bullseye_robot import Robot


class AlgoApp(ABC):
    def __init__(self, obstacles):
        self.grid = Grid(obstacles)
        self.robot = Robot(self.grid)

    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def execute(self):
        pass


class AlgoMinimal(AlgoApp):
    """
    Minimal app to just calculate a path and then send the commands over.
    """
    def __init__(self, obstacles):
        super().__init__(obstacles)

    def init(self):
        pass

    def execute(self):
        pass