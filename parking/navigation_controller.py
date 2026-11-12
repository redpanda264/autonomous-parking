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
            print("xs:", [round(x, 2) for x in pathx]) # changed from rx since it wasn't defined. not sure if that's correct
            print("ys:", [round(y, 2) for y in pathy]) # ry

        self.path = AStarPath(
            x=pathx,
            y=pathy
        )

        # path index
        self.pth_i = 0
        self.pth_i = closest_nondecreasing_path_index(self)

    # pure pursuit
    def control(self):

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

    
    