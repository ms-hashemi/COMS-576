import math
from geometry import is_inside_circle


class Obstacle:
    def contain(self, s):
        return False


class CircularObstacle(Obstacle):
    """A class representing a circular obstacle"""

    def __init__(self, center, radius, theta_lim):
        self.center = center
        self.radius = radius
        self.theta_min = theta_lim[0]
        self.theta_max = theta_lim[1]

    def get_boundaries(self):
        """Return the list of coordinates (x,y) of the boundary of the obstacle"""
        num_theta = 100
        theta_inc = (self.theta_max - self.theta_min) / num_theta
        theta_range = [self.theta_min + theta_inc * i for i in range(num_theta + 1)]
        return [
            (
                self.radius * math.cos(theta) + self.center[0],
                self.radius * math.sin(theta) + self.center[1],
            )
            for theta in theta_range
        ]

    def contain(self, s):
        """Return whether a point s is inside this obstacle"""
        return is_inside_circle(self.center, self.radius, s)


class WorldBoundary2D(Obstacle):
    """A class representing the world"""

    def __init__(self, xlim, ylim):
        self.xmin = xlim[0]
        self.xmax = xlim[1]
        self.ymin = ylim[0]
        self.ymax = ylim[1]

    def contain(self, s):
        """Return True iff the given point is not within the boundary (i.e., the point is
        "in collision" with an obstacle.).
        """
        return (
            s[0] < self.xmin or s[0] > self.xmax or s[1] < self.ymin or s[1] > self.ymax
        )


def construct_circular_obstacles(dt):
    r = 1 - dt  # the radius of the circle
    c = [(0, -1), (0, 1)]  # the center of each circle
    t = [(0, math.pi), (-math.pi, 0)]  # range of theta of each circle
    obstacles = []
    for i in range(len(c)):
        obstacles.append(CircularObstacle(c[i], r, t[i]))
    return obstacles
