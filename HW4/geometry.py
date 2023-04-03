import copy
import math


def get_nearest_point_on_line(q1_line, q2_line, alpha_i, delta_q):
    """
    Auxiliary function to find the nearest point to "alpha_i" on the "q1_line-q2_line" line segment

    @return: a tuple of the nearest point (a tuple itself) on the line segment and its distance (a float) to "alpha_i"
    """
    direction = []
    length = get_euclidean_distance(q1_line, q2_line)
    for i in range(len(q1_line)):
        direction.append((q2_line[i]-q1_line[i])/length) 
    i = 0
    qn = list(copy.deepcopy(q1_line))
    nearest_point = [[0, 0], math.inf]
    length2 = 0
    while length > 0:
        if length < delta_q and i > 0:
            for j in range(len(q2_line)):
                qn[j] = q2_line[j]
            length2 = get_euclidean_distance(qn, alpha_i)
            if length2 < nearest_point[1]:
                return (tuple(qn), length2)
            break
        else:
            for j in range(len(q1_line)):
                qn[j] = q1_line[j] + i * delta_q * direction[j]
            length2 = get_euclidean_distance(qn, alpha_i)
            if length2 < nearest_point[1]:
                nearest_point = [copy.deepcopy(qn), length2]
            length = length - delta_q
            i = i + 1
    return (tuple(nearest_point[0]), nearest_point[1])


def get_euclidean_distance(q1_line, q2_line):
    """Returns the euclidean distance between q1_line and q2_line"""
    length = 0
    for i in range(len(q1_line)):
        length = length + (q2_line[i]-q1_line[i])**2
    return length**0.5


def is_inside_circle(q, center, radius):
    """Checks whether "q" is inside/on the circle defined by "center" and "radius" """
    return get_euclidean_distance(q, center) <= radius


class obstacle(object):
    """A base class to define obstacle objects"""
    def get_boundary(self):
        raise NotImplementedError
    
    def contain(self, robot):
        raise NotImplementedError
    

class circular_obstacle(obstacle):
    """The circle subclass of "obstacle" """
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
    
    def get_boundary(self):
        degree_step = 1
        boundary = [ [0] * 2 for i in range(int(360/degree_step))]
        for theta in range(int(360/degree_step)):
            boundary[theta] = [self.center[0] + self.radius * math.cos(theta*(math.pi/180)), self.center[1] + self.radius * math.sin(theta*(math.pi/180))]
        boundary.append(boundary[0])
        return boundary
    
    def contain(self, q):
        return get_euclidean_distance(self.center, q) <= self.radius


class world_2D_obstacle(obstacle):
    """The 2D world subclass of "obstacle" (the world is treated as a special obstacle in this project)"""
    def __init__(self, origin, width, height):
        self.origin = origin
        self.width = width
        self.height = height
    
    def get_bounds(self):
        return [(self.origin[0]-self.width/2, self.origin[0]+self.width/2), (self.origin[0]-self.height/2, self.origin[0]+self.height/2)]
    
    def get_boundary(self):
        return [
                [self.origin[0] - self.width/2, self.origin[0] - self.height/2],
                [self.origin[0] + self.width/2, self.origin[0] - self.height/2],
                [self.origin[0] + self.width/2, self.origin[0] + self.height/2],
                [self.origin[0] - self.width/2, self.origin[0] + self.height/2],
                [self.origin[0] - self.width/2, self.origin[0] - self.height/2]
        ]

    def contain(self, q):
        boundary = self.get_bounds()
        return q[0] <= boundary[0][1] and q[0] >= boundary[0][0] and q[1] <= boundary[1][1] and q[1] >= boundary[1][0]

