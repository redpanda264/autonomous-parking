import carla
import math
import time
from dataclasses import dataclass
from parking import a_star

# dataclass to hold path data
@dataclass
class AStarPath:
    x: list[float]
    y: list[float]


class NavigationController:
    def __init__(self, ego, world):
        self.ego = ego
        self.world = world


    def control(self):
        return
    