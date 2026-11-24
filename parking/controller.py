import carla
import math
import time
from dataclasses import dataclass
from parking.reeds_shepp_path_planning import reeds_shepp_path_planning
from parking.staging_transform import get_target_transform
from utils.path_utils import closest_nondecreasing_path_index, lookahead_index

# dataclass to hold path data
@dataclass
class SheppPath:
    x: list[float]
    y: list[float]
    yaw: list[float]
    curves: list[str]
    lengths: list[float]
    segment_idxs: list[int]
    directions: list[int]

class Controller:
    def __init__(self, ego, world):
        self.ego = ego
        self.world = world

        # x: + left, - right
        # y: + up, - down

        # target parking spot from camera
        self.target_tf = get_target_transform()

        # control parameters to perform correction if the vehicle is not aligned with the parking spot
        self.final_alignment_distance = 1.0
        self.position_tolerance = 0.2
        self.heading_tolerance = math.radians(2.0)
        self.parked = False
        self.alignment_failed = False
        self.correction_phase = None
        self.correction_attempts = 0
        self.max_correction_attempts = 3
        self.correction_pull_forward = 1.5  # meters along the parking heading
        self.correction_reverse = True
        self.correction_started_at = None
        target_yaw = math.radians(self.target_tf.rotation.yaw)
        self.target_forward = (math.cos(target_yaw), math.sin(target_yaw))


        self.aisle_rotation = carla.Rotation(yaw=90.0) # hard coded for now, should be calculated from aisle

        # TODO: Hard-coding wheelbase for now to calculate steering angle, Change later.
        self.wheelbase = 1.571

        # use reeds-shepp curve, calculate once when in staging position
        physics = ego.get_physics_control()


        # check wheel max steer
        for wheel in physics.wheels:
            if wheel.max_steer_angle > 0:
                self.max_steer_angle = math.radians(wheel.max_steer_angle)
                break

        ego_tf = ego.get_transform()

        # Hardcoded, should approximately match the x and y distance from the staging location to the target parking spot
        min_turning_radius = 4.8 # meters
        max_curvature = 1.0 / min_turning_radius

        # Reeds-Shepp path algorithm from PythonRobotics
        path_x, path_y, path_yaw, path_curvetype, path_length, path_segment_idxs, path_directions = reeds_shepp_path_planning(
            ego_tf.location.x, ego_tf.location.y, math.radians(ego_tf.rotation.yaw),
            self.target_tf.location.x, self.target_tf.location.y, target_yaw,
            max_curvature, step_size=0.2
        )

        # error checking
        if path_x is None or path_y is None or path_yaw is None or path_curvetype is None or path_length is None or path_segment_idxs is None or path_directions is None:
            raise RuntimeError("Path planning failed")


        if any(direction > 0 for direction in path_directions):
            raise RuntimeError("This parking controller requires a reverse-only approach")

        self.path = SheppPath(
            x=path_x,
            y=path_y,
            yaw=path_yaw,
            curves=path_curvetype,
            lengths=path_length,
            segment_idxs=path_segment_idxs,
            directions=path_directions
        )

        # path index
        self.pth_i = 0
        self.pth_i = closest_nondecreasing_path_index(self)


        #diagnostic path values
        print(f"Path X: {self.path.x}")
        print("----------------------------------------------")
        print(f"Path Y: {self.path.y}")
        # print("----------------------------------------------")
        # print(f"Path Yaw: {self.path.yaw}")
        # print("----------------------------------------------")
        # print(f"Path curves: {self.path.curves}")
        # print("----------------------------------------------")
        # print(f"Path lengths: {self.path.lengths}")
        # print("----------------------------------------------")
        # print(f"Path segment indices: {self.path.segment_idxs}")
        print("----------------------------------------------")
        print(f"Path directions: {self.path.directions}")
        print("----------------------------------------------")
        print(f"current ego location: {ego_tf.location.x}, {ego_tf.location.y}")
        print("----------------------------------------------")
        print(f"current yaw: {ego_tf.rotation.yaw}")



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
            print("Reached target parking spot.")
            return True

        curve_range = 5 # how many waypoints ahead to look for curvature estimation
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
        if (len(self.path.x) - self.pth_i) <= 5:
            kappa = 0.0

        # kinematic bicycle model (convert curvature from pure pursuit to steering angle)
        delta = math.atan(self.wheelbase*kappa)
        steering_angle = delta / self.max_steer_angle
        steering_angle = -steering_angle
        steering_angle = max(-1, min(1, steering_angle))

        # reverse = seg_direction < 0
        # might have to change throttle
        control = carla.VehicleControl(throttle=0.18, steer=steering_angle, brake=0.0, reverse=True)
        self.ego.apply_control(control)
