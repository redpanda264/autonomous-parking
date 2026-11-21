import carla
import math
import time
from dataclasses import dataclass
from parking import a_star
from parking import controller
from utils.path_utils import closest_nondecreasing_path_index, lookahead_index

# dataclass to hold path data
@dataclass
class AStarPath:
    x: list[float]
    y: list[float]


# TODO: fix start inside obstacle/outside region error

class NavigationController:
    def __init__(self, ego, world):
        self.ego = ego
        self.world = world

        # TODO: Hard-coding wheelbase for now to calculate steering angle, this code is inconsistent. Change later.
        self.wheelbase = 1.571
        physics = ego.get_physics_control()
        for wheel in physics.wheels:
            if wheel.max_steer_angle > 0:
                self.max_steer_angle = math.radians(wheel.max_steer_angle)
                break
        self.aisle_rotation = carla.Rotation(yaw=90.0) # evil line that is suddenly required - rotation of the aisle where goal parking spot is located

        #TODO: figure out whatever madness is happening here
        self.target_tf = carla.Transform( # THIS IS THE PARKING SPOT
            carla.Location(x=-28.5, y=-38.8, z=0.5), # set to a spot in the A row
            carla.Rotation(yaw=0),
        )

        staging_loc = carla.Transform( # THIS IS THE STAGING LOCATION
            self.target_tf.location + carla.Location(x=-7.0, y=-5.0),
            carla.Rotation(yaw=90.0)
        )

        # final staging transform
        '''
        self.staging_tf = carla.Transform(
            staging_loc,
            self.aisle_rotation
        )
        '''
        self.staging_tf = staging_loc

        start_tf = self.ego.get_transform()

        # A* setup
        resolution = 0.5  # grid resolution in meters
        robot_radius = 1.0  # robot radius in meters
        parking_spots = ( # these are hardcoded bounding boxes that represent the parking spots FROM RIGHT TO LEFT, facing the colored boxes
                a_star.corners_to_rectangle((0.5,-16.0),(13.6,-46.5)),
                a_star.corners_to_rectangle((-16.5,-16.0),(-2.5,-46.5)),
                a_star.corners_to_rectangle((-32.5,-16.0),(-18.9,-46.5)))
        
        bounds = (-40.0, 20.0, -50.0, -10.0) # this covers the ENTIRE parking lot from minX to maxX, minY to maxY in that order
        obstacle_x, obstacle_y = a_star.rectangles_to_points(parking_spots, step = resolution * 0.5)

        ### A* BOUNDS DEBUG LINES HERE ###

        world.debug.draw_point(self.target_tf.location + carla.Location(z=0.5), size=0.1, color=carla.Color(255, 0, 0), life_time=60.0) # parking spot
        world.debug.draw_point(staging_loc.location + carla.Location(z=0.5), size=0.1, color=carla.Color(0, 255, 0), life_time=60.0) # staging spot

        for cx, cy, L, W, yaw in parking_spots: # draws bounding boxes for spots
            world.debug.draw_box(carla.BoundingBox(carla.Location(cx, cy, 0.5), carla.Vector3D(L / 2, W / 2, 0.5)),
                                 carla.Rotation(yaw=yaw), 0.05, carla.Color(255, 0, 0), life_time=60.0)
        
        for cx, cy, L, W, yaw in (a_star.corners_to_rectangle((bounds[0],bounds[2]),(bounds[1],bounds[3])),): # draws complete area
            world.debug.draw_box(carla.BoundingBox(carla.Location(cx, cy, 0.5), carla.Vector3D(L / 2, W / 2, 0.5)),
                                 carla.Rotation(yaw=yaw), 0.05, carla.Color(0, 0, 255), life_time=60.0)
        
        ### A* BOUNDS DEBUG LINES END ###

        #A* execution
        astar_planner = a_star.AStarPlanner(obstacle_x, obstacle_y, resolution, robot_radius, bounds)
        world.wait_for_tick()

        print("start: (" + str(start_tf.location.x) + ", " + str(start_tf.location.y) + ")")
        print("goal: (" + str(self.staging_tf.location.x) + ", " +  str(self.staging_tf.location.y) + ")")

        pathx, pathy = astar_planner.planning(
            start_tf.location.x, start_tf.location.y,
            self.staging_tf.location.x, self.staging_tf.location.y
        )

        if (pathx and pathy):
            print("xs:", [round(x, 2) for x in pathx]) # changed from rx since it wasn't defined. not sure if that's correct
            print("ys:", [round(y, 2) for y in pathy]) # ry o ty
        else:
            raise ValueError("A* path is invalid")

        self.path = AStarPath(
            x=pathx,
            y=pathy
        )

        # path index
        self.pth_i = 0
        # self.pth_i = closest_nondecreasing_path_index(self)

    # pure pursuit
    def control(self):

        # current ego vehicle location/yaw
        ego_loc = self.ego.get_transform().location
        ego_tf = self.ego.get_transform()
        ego_yaw = math.radians(ego_tf.rotation.yaw)
        reverse_yaw = ego_yaw - math.pi

        self.pth_i = closest_nondecreasing_path_index(self)

        if self.pth_i >= len(self.path.x) - 1:
            control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, reverse=True)
            self.ego.apply_control(control)
            print("Reached staging area, transitioning to parking controller...")
            #TODO: implement a state change to transition to the parking controller instead of just stopping the vehicle forever
            return True

        curve_range = 15 # how many waypoints ahead to look for curvature estimation
        future_idx = min(self.pth_i + curve_range, len(self.path.x) - 1)

        # estimate path curve sharpness
        dx_future = self.path.x[future_idx] - ego_loc.x
        dy_future = self.path.y[future_idx] - ego_loc.y
        future_heading = math.atan2(dy_future, dx_future)
        angle_diff = abs(future_heading - reverse_yaw)
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        angle_diff = abs(angle_diff)

        min_lookahead = 0.7
        max_lookahead = 1.8

        lookahead_dist = max(min_lookahead, max_lookahead - (angle_diff * 0.8)) # dynamic lookahead distance based on curve sharpness - angle_diff multiplier changes dropoff speed
        lookahead_i = lookahead_index(self, lookahead_dist)

        # find heading error (alpha) and normalize to [-pi, pi]
        dx = self.path.x[lookahead_i] - ego_loc.x
        dy = self.path.y[lookahead_i] - ego_loc.y

        # subtract math.pi from ego_yaw so the error angle alpha is calculated relative to the rear bumper
        alpha = math.atan2(dy, dx) - reverse_yaw
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        Ld = math.hypot(dx, dy) # lookahead distance

        # minimum 0.5m lookahead distance
        Ld = max(0.5, Ld)

        print(f"pth_i: {self.pth_i}")
        print(f"lookahed idx: {lookahead_i}")
        print(f"lookahed dist: {Ld}")
        
        kappa = 2 * math.sin(alpha) / Ld # curvature

        # fix for wheels sharply turning at the end. set curvature to 0 for the last few waypoints
        if (len(self.path.x) - self.pth_i) <= 3:
            kappa = 0.0

        # kinematic bicycle model (convert curvature from pure pursuit to steering angle)
        delta = math.atan(self.wheelbase*kappa)
        steering_angle = delta / self.max_steer_angle
        steering_angle = -steering_angle
        steering_angle = max(-1, min(1, steering_angle))

        # reverse = seg_direction < 0
        # might have to change throttle
        control = carla.VehicleControl(throttle=0.25, steer=steering_angle, brake=0.0, reverse=False)
        self.ego.apply_control(control)

    
    