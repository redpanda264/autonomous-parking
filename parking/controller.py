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
            carla.Location(x=-13, y=-27.2),
            carla.Rotation(yaw=180.0),
        )

        initial_ego_tf = ego.get_transform()

        # assume car is aligned with the aisle
        self.aisle_rotation = carla.Rotation(yaw=initial_ego_tf.rotation.yaw)

        # -- Calculate staging position --
    

        # rotate target transform to align with ego and aisle
        staging_basis_tf = carla.Transform(
            self.target_tf.location,
            self.aisle_rotation
        )

        # move the rotated target transform right and backwards
        staging_loc = staging_basis_tf.transform(
            carla.Location(x=6.0, y=5.0, z=0.0)
        )

        # final staging transform
        self.staging_tf = carla.Transform(
            staging_loc,
            self.aisle_rotation
        )

        # spawn ego at the staging position for testing
        ego.set_transform(self.staging_tf)
        world.wait_for_tick()

        # TODO: use reeds-shepp curve, calculate once when in staging position

        ego_tf = ego.get_transform()
        min_turning_radius = 5.0 # meters
        max_curvature = 1.0 / min_turning_radius

        # Reeds-Shepp path algorithm from PythonRobotics
        path_x, path_y, path_yaw, path_curvetype, path_length = reeds_shepp_path_planning(
            ego_tf.location.x, ego_tf.location.y, math.radians(ego_tf.rotation.yaw),
            self.target_tf.location.x, self.target_tf.location.y, math.radians(self.target_tf.rotation.yaw),
            max_curvature, step_size=0.2
        )

        # error checking
        if path_x is None or path_y is None or path_yaw is None or path_curvetype is None or path_length is None:
            raise RuntimeError("Path planning failed")

        self.path = SheppPath(
            x=path_x,
            y=path_y,
            yaw=path_yaw,
            curves=path_curvetype,
            lengths=path_length
        )

        # path index
        self.pth_i = 0

        # diagnostic
        # print(f"Path X: {self.path.x}")
        # print("----------------------------------------------")
        # print(f"Path Y: {self.path.y}")
        # print("----------------------------------------------")
        # print(f"Path Yaw: {self.path.yaw}")



    def control(self):

        

        # TODO: control algorithm to minimize distance to curve

        # current ego vehicle location/yaw
        ego_loc = self.ego.get_transform().location
        ego_tf = self.ego.get_transform()

        print(
            f"index={self.pth_i}, "
            f"ego=({ego_loc.x:.2f}, {ego_loc.y:.2f}), "
            f"waypoint=({self.path.x[self.pth_i]:.2f}, {self.path.y[self.pth_i]:.2f}), "
            f"distance={math.dist((ego_loc.x, ego_loc.y), (self.path.x[self.pth_i], self.path.y[self.pth_i])):.2f}")

        if self.pth_i >= len(self.path.x):
            print("Reached parking spot")
            control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
            self.ego.apply_control(control)
            return

        # if ( self.pth_i >= len(self.path.x) - 1 or                        # at end of waypoint list
        #     math.dist((ego_loc.x, ego_loc.y), (self.path.x[self.pth_i], self.path.y[self.pth_i])) # dist to current waypoint
        #     <
        #     math.dist((ego_loc.x, ego_loc.y), (self.path.x[self.pth_i+1], self.path.y[self.pth_i+1])) #dist to next waypoint
        # ):
        #     # on the last waypoint, or closer to current waypoint, keep backing up in same direction
        #     pass
        # else:
        #     # closer to the next waypoint, change direction
        #     self.pth_i += 1

        if math.dist((ego_loc.x, ego_loc.y), (self.path.x[self.pth_i], self.path.y[self.pth_i])) < 0.1:
            # if within 1m of the current waypoint, move to the next one
            self.pth_i += 1
            return
        else:
            dx = self.path.x[self.pth_i] - ego_loc.x
            dy = self.path.y[self.pth_i] - ego_loc.y

            right_vec = ego_tf.get_right_vector()
            forward_vec = ego_tf.get_forward_vector()

            lateral = dx * right_vec.x + dy * right_vec.y
            longitudinal = dx * forward_vec.x + dy * forward_vec.y
            gain = 0.5 # steering strength

            angle = math.atan2(lateral, max(abs(longitudinal), 0.1))
            steer = max(-1.0, min(1.0, -gain * angle))  # reverse: flip the sign


        print(f"index: {self.pth_i}, distance: {math.dist((ego_loc.x, ego_loc.y), (self.path.x[self.pth_i], self.path.y[self.pth_i]))}, steer: {steer:.2f}")
        control = carla.VehicleControl(throttle=0.3, steer=steer, reverse=True)
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