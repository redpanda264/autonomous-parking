"""

A* grid planning

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
Based on Atsushi Sakai's implementation (https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py)
    authors: Atsushi Sakaib (@Atsushi_twi)
             Nikos Kanargias (nkana@tee.gr)

"""

import math
import matplotlib.pyplot as plt
import heapq
import numpy as np

class AStarPlanner:

    # initialize
    def __init__(self, ox, oy, resolution, rr, bounds, is_blocked=None):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        
        self.min_x, self.max_x, self.min_y, self.max_y = bounds
        if not (self.max_x > self.min_x and self.max_y > self.min_y):
            raise ValueError("bounds must be (min_x, max_x, min_y, max_y) in CARLA world coords with max > min; "
                f"got {bounds}")

        self.x_width = abs(int(round((self.max_x - self.min_x) / resolution))) + 1
        self.y_width = abs(int(round((self.max_y - self.min_y) / resolution))) + 1

        self.is_blocked = is_blocked
        self.motions = self.get_motion_model()
        
        self.obstacle_map = None
        self.calc_obstacle_map(ox, oy)
    
    # set of functions to go from grid <--> world
    def to_index(self, x, y):
        return (int(round((x - self.min_x) / self.resolution)),
                int(round((y - self.min_y) / self.resolution)))
 
    def to_world(self, ix, iy):
        return ix * self.resolution + self.min_x, iy * self.resolution + self.min_y
    
    # obstacle map stuff
    def calc_obstacle_map(self, ox, oy):
        grid = np.zeros((self.x_width, self.y_width), dtype=bool)
        if len(ox) == 0:
            return grid
        r = int(math.ceil(self.rr / self.resolution))
        offsets = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
                   if math.hypot(dx, dy) * self.resolution <= self.rr]
        ix = np.round((np.asarray(ox) - self.min_x) / self.resolution).astype(int)
        iy = np.round((np.asarray(oy) - self.min_y) / self.resolution).astype(int)
        for i, j in set(zip(ix.tolist(), iy.tolist())):
            for dx, dy in offsets:
                a, b = i + dx, j + dy
                if 0 <= a < self.x_width and 0 <= b < self.y_width:
                    grid[a, b] = True
        
        self.obstacle_map = grid
    
    def blocked(self, node):
        ix, iy = node
        if not (0 <= ix < self.x_width and 0 <= iy < self.y_width):
            return True
        if self.obstacle_map[ix, iy]:
            return True
        if self.is_blocked is not None:
            if node not in self._extra_cache:
                self._extra_cache[node] = bool(self.is_blocked(*self.to_world(ix, iy)))
            return self._extra_cache[node]
        return False

    # search
    def planning(self, sx, sy, gx, gy):
        """
        Plans a route from a given start to a given goal based on the current state of the A* planner

        sx: x position of the start
        sy: y position of the start
        gx: x position of the goal
        gy: y position of the goal
        """
        start, goal = self.to_index(sx, sy), self.to_index(gx, gy)

        # safety
        for name, (px, py), node in (("start", (sx, sy), start), ("goal", (gx, gy), goal)):
            if self.blocked(node):
                ix, iy = node
                b = ""
                if not (0 <= ix < self.x_width and 0 <= iy < self.y_width):
                    b = f"outside bounds x[{self.min_x}, {self.max_x}] y[{self.min_y}, {self.max_y}]"
                elif self.obstacle_map[ix, iy]:
                    wx, wy = self.to_world(ix, iy)
                    b = print(f"inside an obstacle or robot radius {self.rr} m inflation (cell center {wx:.2f}, {wy:.2f}")
                
                a = f"{name} {(px, py)} exploded bc"
                print(f"{a} {b}")
                return (None, None)
 
        def h(n):
            return math.hypot(n[0] - goal[0], n[1] - goal[1]) * self.resolution
 
        g_cost = {start: 0.0}
        parent = {start: None}
        closed = set()
        heap = [(h(start), 0, start)]
        tie = 0
 
        while heap:
            _, _, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            closed.add(cur)
            if cur == goal:
                break
            for dx, dy, c in self.motions:
                nxt = (cur[0] + dx, cur[1] + dy)
                if nxt in closed or self.blocked(nxt):
                    continue
                # no corner cutting on diagonals
                if dx != 0 and dy != 0 and (self.blocked((cur[0] + dx, cur[1])) or
                                            self.blocked((cur[0], cur[1] + dy))):
                    continue
                ng = g_cost[cur] + c * self.resolution
                if ng < g_cost.get(nxt, float("inf")):
                    g_cost[nxt] = ng
                    parent[nxt] = cur
                    tie += 1
                    heapq.heappush(heap, (ng + h(nxt), tie, nxt))
        else:
            print("no path found")
            return None, None
 
        rx, ry = [], []
        n = goal
        while n is not None:
            x, y = self.to_world(*n)
            rx.append(x)
            ry.append(y)
            n = parent[n]
        return rx[::-1], ry[::-1]

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1.0],
                  [0, 1, 1.0],
                  [-1, 0, 1.0],
                  [0, -1, 1.0],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

def sample_box(cx, cy, ex, ey, yaw, step):
    """
    Fills an oriented rectangle (center, half extents, and yaw) with points
    """
    xs = np.arange(-ex, ex + step, step)
    ys = np.arange(-ey, ey + step, step)
    gx, gy = np.meshgrid(xs,ys)
    
    # adjustments made if yaw exists
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    
    wx = cx + gx.ravel() * c - gy.ravel() * s
    wy = cy + gx.ravel() * s + gy.ravel() * c
    
    return wx, wy

@staticmethod
def corners_to_rectangle(corner1, corner2, yaw=0.0):
    """
    Given two corners (x, y) of a rectangle, output an iterable (x, y, width, height, yaw)
    such that: x, y = the center of the rectangle
    corner1 = one corner of the rectangle
    corner2 = the opposite corner of the rectangle, specifically opposite
    yaw = yaw
    """
    (x1, y1), (x2, y2) = corner1, corner2
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0, abs(x2 - x1), abs(y2 - y1), yaw

@staticmethod
def rectangles_to_points(rects, step=0.5):
    """
    rects = iterable of multiple (x, y, width, height)s
    """
    ox, oy = [], []
    for cx, cy, width, height, yaw in rects:
        wx, wy = sample_box(cx, cy, width / 2.0, height / 2.0, yaw, step)
        ox += wx.tolist()
        oy += wy.tolist()
    return ox, oy

"""
# test function !, basically ported this over to navigation_controller actually lol
def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = -16.0
    sy = -12.0
    gx = -17.8
    gy = -48.0

    # example state of the lot
    parking_spots = (
        AStarPlanner.corners_to_rectangle((0.0,-14.0),(13.0,-46.5)),
        AStarPlanner.corners_to_rectangle((-15.5,-14.0),(-2.5,-46.5)),
        AStarPlanner.corners_to_rectangle((-31.0,-14.0),(-18.5,-46.5)))
    bounds = (-37, 17, -48, -11)

    # state of the map
    grid_size = 0.5
    robot_radius = 1.0

    ox, oy = rectangles_to_points(parking_spots, step = grid_size * 0.5)

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, bounds)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if rx and ry:
        print("xs:", [round(x, 2) for x in rx])
        print("ys:", [round(y, 2) for y in ry])


if __name__ == '__main__':
    main()
"""