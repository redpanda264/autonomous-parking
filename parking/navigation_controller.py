import carla
import math
import time
from dataclasses import dataclass
from parking import a_star
from controller import closest_nondecreasing_path_index

# dataclass to hold path data
@dataclass
class AStarPath:
    x: list[float]
    y: list[float]


class NavigationController:
    def __init__(self, ego, world):
        self.ego = ego
        self.world = world

        #TODO: change this to a staging area: goal
        self.target_tf = carla.Transform(
            carla.Location(x=-23, y=-38.3, z=0.5),
            carla.Rotation(yaw=0),
        )

        staging_loc = self.target_tf.transform(
            carla.Location(x=5.0, y=5.0, z=0.0)
        )

        # final staging transform
        self.staging_tf = carla.Transform(
            staging_loc,
            self.aisle_rotation
        )

        start_tf = self.ego.get_transform()

        #TODO: change these variables to the actual values
        resolution = 0.5  # grid resolution in meters
        robot_radius = 1.0  # robot radius in meters
        obstacle_x = [0, 10, 20]  # x coordinates of obstacles
        obstacle_y = [0, 10, 20]  # y coordinates of obstacles

        astar_planner = a_star.AStarPlanner(obstacle_x, obstacle_y, resolution, robot_radius)
        pathx, pathy = astar_planner.planning(
            start_tf.location.x, start_tf.location.y,
            self.staging_tf.location.x, self.staging_tf.location.y
        )

        self.path = AStarPath(
            x=pathx,
            y=pathy
        )

        # path index
        self.pth_i = 0
        self.pth_i = self.closest_nondecreasing_path_index()


    def control(self):
        return
    