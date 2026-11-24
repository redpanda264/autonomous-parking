import math

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