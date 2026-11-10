import carla
import math
import time
from dataclasses import dataclass
from parking import a_star
from parking import controller

# dataclass to hold path data
@dataclass
class AStarPath:
    x: list[float]
    y: list[float]


class NavigationController:
    def __init__(self, ego, world):
        self.ego = ego
        self.world = world

        self.aisle_rotation = carla.Rotation(yaw=90.0) # evil line that is suddenly required

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

        # A* setup
        resolution = 0.5  # grid resolution in meters
        robot_radius = 1.0  # robot radius in meters
        parking_spots = ( # these are hardcoded bounding boxes that represent the parking spots
                a_star.corners_to_rectangle((0.0,-15.0),(13.0,-46.5)),
                a_star.corners_to_rectangle((-15.5,-15.0),(-2.5,-46.5)),
                a_star.corners_to_rectangle((-31.0,-15.0),(-18.5,-46.5)))
        
        bounds = (-40.0, 20.0, -50.0, -10.0) # this covers the ENTIRE parking lot from minX to maxX, minY to maxY in that order
        
        # TESTING LINE TO SEE THE OCCUPIED PARKING SPOTS
        for cx, cy, L, W, yaw in parking_spots:
            world.debug.draw_box(carla.BoundingBox(carla.Location(cx, cy, 0.5), carla.Vector3D(L / 2, W / 2, 0.5)),
                                 carla.Rotation(yaw=yaw), 0.05, carla.Color(255, 0, 0), 60.0)
        
        for cx, cy, L, W, yaw in (a_star.corners_to_rectangle((bounds[0],bounds[2]),(bounds[1],bounds[3])),):
            world.debug.draw_box(carla.BoundingBox(carla.Location(cx, cy, 0.5), carla.Vector3D(L / 2, W / 2, 0.5)),
                                 carla.Rotation(yaw=yaw), 0.05, carla.Color(0, 0, 255), 60.0)

        obstacle_x, obstacle_y = a_star.rectangles_to_points(parking_spots, step = resolution * 0.5)

        #A* execution
        astar_planner = a_star.AStarPlanner(obstacle_x, obstacle_y, resolution, robot_radius, bounds)

        pathx, pathy = astar_planner.planning(
            start_tf.location.x, start_tf.location.y,
            self.staging_tf.location.x, self.staging_tf.location.y
        )

        if (pathx and pathy):
            print("xs:", [round(x, 2) for x in rx])
            print("ys:", [round(y, 2) for y in ry])

        self.path = AStarPath(
            x=pathx,
            y=pathy
        )

        # path index
        self.pth_i = 0
        #self.pth_i = controller.closest_nondecreasing_path_index()

    def control(self):
        return
    