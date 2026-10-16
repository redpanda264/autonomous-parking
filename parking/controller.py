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
        #self.target_tf = carla.Transform(
        #    carla.Location(x=-13, y=-38.3, z=0.5),
        #    carla.Rotation(yaw=0.0),
        #)

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

        # physics = self.ego.get_physics_control()
        # wheels = physics.wheels

        # front_x = (wheels[0].position.x + wheels[1].position.x) / 2
        # rear_x = (wheels[2].position.x + wheels[3].position.x) / 2

        # self.wheelbase = abs(front_x - rear_x)

        # print(f"Wheelbase: {self.wheelbase:.3f} m")

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

        # diagnostic
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

        # print("Staging:", ego_tf.location.x, ego_tf.location.y)
        # print("Target:", self.target_tf.location.x, self.target_tf.location.y)
        # print("staging yaw:", self.staging_tf.rotation.yaw)

        # print("First waypoint:", path_x[0], path_y[0])
        # print("Last waypoint:", path_x[-1], path_y[-1])



    def control(self):

        # pure pursuit control algorithm to follow path planning

        # current ego vehicle location/yaw
        ego_loc = self.ego.get_transform().location
        ego_tf = self.ego.get_transform()
        ego_yaw = math.radians(ego_tf.rotation.yaw)

        if math.dist((ego_loc.x, ego_loc.y), (self.path.x[self.pth_i], self.path.y[self.pth_i])) < 0.3:
            # next waypoint
            self.pth_i += 1

        # if reached end of waypoints
        if self.pth_i >= len(self.path.x):
            print("Reached parking spot")
            control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
            self.ego.apply_control(control)
            return

        

        # control

        lookAheadDist = 1.0 # hard-coded distance to look ahead point
        lookAheadIdx = len(self.path.x) - 1
        for i in range(self.pth_i, len(self.path.x)):
            if math.dist((ego_loc.x, ego_loc.y), (self.path.x[i], self.path.y[i])) >= lookAheadDist:
                lookAheadIdx = i
                break

        # find heading error (alpha) and normalize to [-pi, pi]
        dx = self.path.x[lookAheadIdx] - ego_loc.x
        dy = self.path.y[lookAheadIdx] - ego_loc.y
        alpha = math.atan2(dy, dx) - ego_yaw
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        Ld = math.hypot(dx, dy) # lookahead distance
        if Ld < 1e-6:
            print("cannot divide by 0")
            return
        
        kappa = 2 * math.sin(alpha) / Ld # curvature

        # kinematic bicycle model (convert curvature from pure pursuit to steering angle)
        delta = math.atan(self.wheelbase*kappa)
        steering_angle = delta / self.max_steer_angle
        steering_angle = max(-1, min(1, steering_angle))

        #reverse = seg_direction < 0
        control = carla.VehicleControl(throttle=0.2, steer=steering_angle, brake=0.0, reverse=True)
        self.ego.apply_control(control)


        # TODO: move driving logic into another controller class

        # testing purposes
        #self.ego.set_transform(self.staging_tf)
        #self.ego.set_transform(self.target_tf)

        # ego_tf = self.ego.get_transform() 

        # relative_x, relative_y = self.get_relative_position()
        # bearing = math.atan2(relative_y, relative_x)
        # steer = max(-0.5, min(0.5, 1.0 * bearing)) # limit steering to [-0.5, 0.5]

        # # testing
        # desired_yaw = self.staging_tf.rotation.yaw
        # current_yaw = ego_tf.rotation.yaw
        # yaw_error = (desired_yaw - current_yaw + 180) % 360 - 180
        # print(f"Heading error: {yaw_error:.1f} degrees")

        # print(f"Relative distance to staging area: x={relative_x:.2f}, y={relative_y:.2f}")

        # if relative_x > 0 and math.hypot(relative_x, relative_y) > 0.5:
        #     # move forward
        #     control = carla.VehicleControl(throttle=0.2, steer=steer, brake=0.0)

        # else:
        #     # stop
        #     control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.8)
        #     print("Reached staging area, stopping vehicle.")

        # self.ego.apply_control(control)




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