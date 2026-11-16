import carla
import math
import time
from dataclasses import dataclass
from parking.reeds_shepp_path_planning import reeds_shepp_path_planning
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

        # hard coded target location for parking spot
        # self.target_tf = carla.Transform(
        #     carla.Location(x=-13, y=-27.2, z=0.5),
        #     carla.Rotation(yaw=180),
        # )

        #different spot
        # self.target_tf = carla.Transform(
        #     carla.Location(x=-13, y=-35.5, z=0.5),
        #     carla.Rotation(yaw=180),
        # )

        #parking spot
        #TODO: FIX THIS. this is not a parking spot
        self.target_tf = carla.Transform( # THIS IS THE PARKING SPOT
            carla.Location(x=-35.5, y=-34.1, z=0.5),
            carla.Rotation(yaw=0),
        )

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

        # assume car is aligned with the aisle
        self.aisle_rotation = carla.Rotation(yaw=90.0) # hard coded for now, should be calculated from aisle

        # move the rotated target transform right and backwards (spot to left)
        staging_loc = self.target_tf.transform(
            carla.Location(x=5.0, y=-5.0, z=0.0)
        )

        #(spot to right)
        # staging_loc = self.target_tf.transform(
        #     carla.Location(x=5.0, y=5.0, z=0.0)
        # )

        # final staging transform
        self.staging_tf = carla.Transform(
            staging_loc,
            self.aisle_rotation
        )


        # TODO: Hard-coding wheelbase for now to calculate steering angle, this code is inconsistent. Change later.
        self.wheelbase = 1.571

        # use reeds-shepp curve, calculate once when in staging position
        physics = ego.get_physics_control()


        # check wheel max steer
        for wheel in physics.wheels:
            if wheel.max_steer_angle > 0:
                self.max_steer_angle = math.radians(wheel.max_steer_angle)
                break

        #TODO: teleport ego, since it's not working. debugging
        ego.set_transform(self.target_tf)
        world.wait_for_tick()

        ego_tf = ego.get_transform()
        #ego_tf = self.staging_tf - if having issue with ego tf
        min_turning_radius = 5.0 # meters
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

        # pure pursuit control algorithm to follow path planning

        # current ego vehicle location/yaw
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        ego_yaw = math.radians(ego_tf.rotation.yaw)
        reverse_yaw = ego_yaw - math.pi

        self.pth_i = closest_nondecreasing_path_index(self)


        # calculate distance to target and heading error
        target_distance = math.hypot(
            ego_loc.x - self.target_tf.location.x,
            ego_loc.y - self.target_tf.location.y,
        )

        # calculate heading error relative to target yaw
        target_yaw = math.radians(self.target_tf.rotation.yaw)
        heading_error =     (target_yaw - ego_yaw + math.pi) % (2 * math.pi) - math.pi
        remaining = ((ego_loc.x - self.target_tf.location.x) * self.target_forward[0]
                     + (ego_loc.y - self.target_tf.location.y) * self.target_forward[1])
        on_final_approach = (target_distance <= self.final_alignment_distance
                             and self.pth_i >= len(self.path.x) - 7)
        aligned = (target_distance <= self.position_tolerance
                   and abs(heading_error) <= self.heading_tolerance)
        passed_target = on_final_approach and remaining <= 0.0

        if self.parked or self.alignment_failed:
            control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, reverse=True)
            self.ego.apply_control(control)
            return
        if aligned or passed_target:
            # Stop first, then verify the pose or begin a physical correction.
            self.correction_phase = "verify" if aligned else "brake_forward"
            self.correction_started_at = time.monotonic()
            self.correct_alignment()
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
        lookahead_i = lookahead_index(self, lookahead_dist)

        # find heading error (alpha) and normalize to [-pi, pi]
        dx = self.path.x[lookahead_i] - ego_loc.x
        dy = self.path.y[lookahead_i] - ego_loc.y
        if on_final_approach:
            # Extend the straight reference beyond the endpoint so heading can
            # still converge when less than a lookahead distance remains.
            lookahead_dist = 0.7
            reference_distance = remaining - lookahead_dist
            dx = self.target_tf.location.x + reference_distance * self.target_forward[0] - ego_loc.x
            dy = self.target_tf.location.y + reference_distance * self.target_forward[1] - ego_loc.y

        # Calculate the pursuit angle relative to the reverse travel direction.
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
        velocity = self.ego.get_velocity()
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        throttle, brake = 0.2, 0.0
        if target_distance < 1.5:
            desired_speed = min(0.45, max(0.22, target_distance * 0.4))
            speed_error = desired_speed - speed
            throttle = min(0.2, max(0.0, speed_error * 0.5))
            brake = min(0.5, max(0.0, -speed_error * 0.5))
        control = carla.VehicleControl(throttle=throttle, steer=steering_angle, brake=brake, reverse=True)
        self.ego.apply_control(control)








    
    #TODO: remove unused methods below, refactor.


    # NEW METHOD: Correct the vehicle's alignment with the parking spot
    def correct_alignment(self):
        """Pull forward along the spot centerline, stop, then reverse into it."""
        tf = self.ego.get_transform()
        dx = tf.location.x - self.target_tf.location.x
        dy = tf.location.y - self.target_tf.location.y
        fx, fy = self.target_forward
        along = dx * fx + dy * fy
        lateral = -dx * fy + dy * fx
        target_yaw = math.radians(self.target_tf.rotation.yaw)
        heading_error = (math.radians(tf.rotation.yaw) - target_yaw + math.pi) % (2 * math.pi) - math.pi
        velocity = self.ego.get_velocity()
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        aligned = (math.hypot(dx, dy) <= self.position_tolerance
                   and abs(heading_error) <= self.heading_tolerance)

        def brake():
            self.ego.apply_control(carla.VehicleControl(
                throttle=0.0, steer=0.0, brake=1.0, reverse=self.correction_reverse))

        # Keep this a small parking adjustment, with bounded retries and time.
        if (abs(lateral) > 0.7 or abs(along) > 3.0
                or abs(heading_error) > math.radians(45.0)
                or time.monotonic() - self.correction_started_at > 90.0):
            brake()
            self.alignment_failed = True
            self.correction_phase = None
            print("Alignment correction stopped: maneuver limit exceeded")
            return

        if self.correction_phase == "verify":
            brake()
            if speed > 0.03:
                return
            if aligned:
                self.parked = True
                self.correction_phase = None
                print("Reached target with parking alignment")
            else:
                self.correction_phase = "brake_forward"
            return

        if self.correction_phase in ("brake_forward", "brake_reverse"):
            brake()
            if speed > 0.03:
                return
            if self.correction_phase == "brake_forward":
                if self.correction_attempts >= self.max_correction_attempts:
                    self.alignment_failed = True
                    self.correction_phase = None
                    print("Alignment correction stopped: retry limit reached")
                    return
                self.correction_attempts += 1
                self.correction_end_x = max(self.correction_pull_forward, along + 1.0)
                self.correction_phase = "forward"
                self.correction_reverse = False
                print(f"Alignment correction {self.correction_attempts}: pull forward "
                      f"(heading error={math.degrees(heading_error):.1f} deg)")
            else:
                self.correction_phase = "reverse"
                self.correction_reverse = True
                print("Alignment correction: reverse into spot")
            return

        reverse = self.correction_phase == "reverse"
        if not reverse and along >= self.correction_end_x:
            self.correction_phase = "brake_reverse"
            brake()
            return
        if reverse and (aligned or along <= 0.0):
            self.correction_phase = "verify"
            brake()
            return

        direction = -1.0 if reverse else 1.0
        # Direction-aware heading and lateral feedback stabilizes the same
        # centerline in either gear; do not apply the pursuit reverse sign here.
        curvature = -1.0 * lateral - direction * 2.5 * heading_error
        steer = math.atan(self.wheelbase * curvature) / self.max_steer_angle
        steer = max(-0.5, min(0.5, steer))
        distance_left = max(0.0, along if reverse else self.correction_end_x - along)
        desired_speed = min(0.4, max(0.18, distance_left * 0.5))
        speed_error = desired_speed - speed
        self.ego.apply_control(carla.VehicleControl(
            throttle=min(0.2, max(0.0, speed_error * 0.5)),
            steer=steer, brake=min(0.5, max(0.0, -speed_error * 0.5)),
            reverse=reverse))



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
