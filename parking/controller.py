import carla
import math
from dataclasses import dataclass
from parking.reeds_shepp_path_planning import reeds_shepp_path_planning

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

        # hard coded target location for parking spot
        self.target_tf = carla.Transform(
            carla.Location(x=-13, y=-27.2, z=0.5),
            carla.Rotation(yaw=180),
        )

        # assume car is aligned with the aisle
        self.aisle_rotation = carla.Rotation(yaw=90.0) # hard coded for now, should be calculated from aisle

        # move the rotated target transform right and backwards
        staging_loc = self.target_tf.transform(
            carla.Location(x=5.0, y=-5.0, z=0.0)
        )

        # final staging transform
        self.staging_tf = carla.Transform(
            staging_loc,
            self.aisle_rotation
        )

        # spawn ego at the staging position for testing
        ego.set_transform(self.staging_tf)
        world.wait_for_tick()


        # TODO: Hard-coding wheelbase for now to calculate steering angle, this code is inconsistent. Change later.
        self.wheelbase = 1.571


        # use reeds-shepp curve, calculate once when in staging position
        physics = ego.get_physics_control()


        # check wheel max steer
        for wheel in physics.wheels:
            if wheel.max_steer_angle > 0:
                self.max_steer_angle = math.radians(wheel.max_steer_angle)
                break



        ego_tf = ego.get_transform()
        min_turning_radius = 5.0 # meters
        max_curvature = 1.0 / min_turning_radius

        # Reeds-Shepp path algorithm from PythonRobotics
        path_x, path_y, path_yaw, path_curvetype, path_length, path_segment_idxs, path_directions = reeds_shepp_path_planning(
            ego_tf.location.x, ego_tf.location.y, math.radians(ego_tf.rotation.yaw),
            self.target_tf.location.x, self.target_tf.location.y, math.radians(self.target_tf.rotation.yaw),
            max_curvature, step_size=0.2
        )

        # error checking
        if path_x is None or path_y is None or path_yaw is None or path_curvetype is None or path_length is None or path_segment_idxs is None or path_directions is None:
            raise RuntimeError("Path planning failed")

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
        self.pth_i = self.closest_nondecreasing_path_index()


        # diagnostic path values
        # print(f"Path X: {self.path.x}")
        # print("----------------------------------------------")
        # print(f"Path Y: {self.path.y}")
        # print("----------------------------------------------")
        # print(f"Path Yaw: {self.path.yaw}")
        # print("----------------------------------------------")
        # print(f"Path curves: {self.path.curves}")
        # print("----------------------------------------------")
        # print(f"Path lengths: {self.path.lengths}")
        # print("----------------------------------------------")
        # print(f"Path segment indices: {self.path.segment_idxs}")
        # print("----------------------------------------------")
        # print(f"Path directions: {self.path.directions}")



    def control(self):

        # pure pursuit control algorithm to follow path planning

        # current ego vehicle location/yaw
        ego_loc = self.ego.get_transform().location
        ego_tf = self.ego.get_transform()
        ego_yaw = math.radians(ego_tf.rotation.yaw)
        reverse_yaw = ego_yaw - math.pi

        self.pth_i = self.closest_nondecreasing_path_index()

        if self.pth_i >= len(self.path.x) - 1:
            control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, reverse=True)
            self.ego.apply_control(control)
            print("Reached target")
            return

        curve_range = 4
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
        lookahead_i = self.lookahead_index(lookahead_dist)

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

        # kinematic bicycle model (convert curvature from pure pursuit to steering angle)
        delta = math.atan(self.wheelbase*kappa)
        steering_angle = delta / self.max_steer_angle
        steering_angle = -steering_angle
        steering_angle = max(-1, min(1, steering_angle))

        #reverse = seg_direction < 0
        control = carla.VehicleControl(throttle=0.2, steer=steering_angle, brake=0.0, reverse=True)
        self.ego.apply_control(control)


    def closest_nondecreasing_path_index(self):
        ego_loc = self.ego.get_transform().location
        min_dist = math.inf
        min_i = self.pth_i

        for i in range(self.pth_i, len(self.path.x)):
            cur_dist = math.dist((ego_loc.x, ego_loc.y), (self.path.x[i], self.path.y[i]))
            if cur_dist <= min_dist:
                min_dist = cur_dist
                min_i = i
            else:
                # stop if distance starts increasing
                break

        return min_i


    def lookahead_index(self, lookahead_dist):
        ego_loc = self.ego.get_transform().location

        for i in range(self.pth_i, len(self.path.x)):
            cur_dist = math.dist((ego_loc.x, ego_loc.y), (self.path.x[i], self.path.y[i]))

            if cur_dist >= lookahead_dist:
                return i

        return len(self.path.x) - 1
        


    def get_relative_position(self):
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        staging_loc = self.staging_tf.location
        
        dx = staging_loc.x - ego_loc.x
        dy = staging_loc.y - ego_loc.y
        dz = staging_loc.z - ego_loc.z
        
        # use ego vectors to find relative distance
        forward = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        relative_x = dx * forward.x + dy * forward.y + dz * forward.z
        relative_y = dx * right.x + dy * right.y + dz * right.z

        return relative_x, relative_y