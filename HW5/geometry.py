import copy
import math
# Most probably, the following are not available conventionally, so plaese install "Shapely"
import shapely # A Python library for set-theoretic analysis and manipulation of planar features
# Importing "numpy" library for math functions ("sin", "cos") and matrix multiplication ("matmul")
import numpy
# For Dubins car path planning
from dubins_path_planner import plan_dubins_path

# Importing this code from HW2 for the transformations of the robot
def get_link_positions(config, W, L, D):
    """Compute the positions of the links and the joints of a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link

    @return: a tuple (joint_positions, link_vertices) where
        * joint_positions is a list [p_1, ..., p_{m+1}] where p_i is the position [x,y] of the joint between A_i and A_{i-1}
        * link_vertices is a list [V_1, ..., V_m] where V_i is the list of [x,y] positions of vertices of A_i
    """
    # TODO: Implement this function

    joint_positions = [] # The positions of all joints in a list with repect to the world frame
    link_vertices = [] # The positions of all vertices of all linkages in a list (of lists) with repect to the world frame
    rot = numpy.zeros((1, 3, 3)) # Initializing the rotation matrix (or matrices) as a numpy matrix (the first one is just a placeholder to be appended by later rotation matrices)
    if len(config) > 0:
        joint_positions.append([0.0, 0.0]) # The absolute or world frame position of the first joint is always at the center of the world frame per convention/problem statement
    # Calculating all linkages' positions in the world frame
    joint_relative = [0.0, 0.0] # The relative or current frame position of the first or '0'th joint
    for i in range(len(config)):
        theta = config[i]
        rot = numpy.append(rot, numpy.array([[[numpy.cos(theta), -numpy.sin(theta), joint_relative[0]], [numpy.sin(theta), numpy.cos(theta), joint_relative[1]], [0.0, 0.0, 1.0]]]), axis = 0)
        joint_relative = [D, 0.0]
        joint = numpy.expand_dims(numpy.array([D, 0.0, 1.0]).T, axis = 1) # The relative or current frame position of the next or 'i+1'th joint
        v_1 = numpy.expand_dims(numpy.array([-(L-D)/2, -W/2, 1.0]).T, axis = 1) # The relative or current frame position of the first vertex of the current linkage
        v_2 = numpy.expand_dims(numpy.array([D+(L-D)/2, -W/2, 1.0]).T, axis = 1) # The relative or current frame position of the second vertex of the current linkage
        v_3 = numpy.expand_dims(numpy.array([D+(L-D)/2, +W/2, 1.0]).T, axis = 1) # The relative or current frame position of the third vertex of the current linkage
        v_4 = numpy.expand_dims(numpy.array([-(L-D)/2, +W/2, 1.0]).T, axis = 1) # The relative or current frame position of the fourth vertex of the current linkage
        # if i == 0:
        #     rot = numpy.append(rot, numpy.array([[[numpy.cos(theta), -numpy.sin(theta), 0], [numpy.sin(theta), numpy.cos(theta), 0], [0.0, 0.0, 1.0]]]), axis = 0)
        # else:
        #     rot = numpy.append(rot, numpy.array([[[numpy.cos(theta), -numpy.sin(theta), D], [numpy.sin(theta), numpy.cos(theta), 0], [0.0, 0.0, 1.0]]]), axis = 0)
        # Applying successive transformations for the current link as number as the current linkage index (stored in "rot")
        for j in range(1, len(rot)):
            # The transformations should begin from the last one (the last relative transformation matrix), so we should multiply by "rot[-j]" instead of "rot[j]"
            joint = numpy.matmul(rot[-j], joint) # At the end of all successive transformations, its value is the absolute or world frame position of the joint.
            v_1 = numpy.matmul(rot[-j], v_1)
            v_2 = numpy.matmul(rot[-j], v_2)
            v_3 = numpy.matmul(rot[-j], v_3)
            v_4 = numpy.matmul(rot[-j], v_4)

        # Appending the absolute position of the current joint to "joint_positions"
        joint_positions.append(joint.flatten()[:-1].tolist())
        # Appending the absolute positions of the current linkage vertices to "link_vertices"
        link_vertices.append([v_1.flatten()[:-1].tolist(), v_2.flatten()[:-1].tolist(), v_3.flatten()[:-1].tolist(), v_4.flatten()[:-1].tolist()])
        # print(joint_positions) # For debugging!
        # print(link_vertices) # For debugging!

    return (joint_positions, link_vertices)
    # raise NotImplementedError


# Importing this code from HW2 for computing Cobs (not used in this project)
def compute_Cobs(O, W, L, D):
    """Compute C-Space obstacles for a 2-link robot

    @type O:   a list of obstacles, where for each i, O[i] is a list [(x_0, y_0), ..., (x_m, y_m)]
               of coordinates of the vertices of the i^th obstacle
    @type W:   float, representing the width of each link
    @type L:   float, representing the length of each link
    @type D:   float, the distance between the two points of attachment on each link

    @return: a list of configurations (theta_1, theta_2) of the robot that leads to a collision
        between the robot and an obstacle in O.
    """
    # TODO: Implement this function
    import numpy # Importing Numpy for math functions (including pi number)
    # Most probably, the following are not available conventionally, so plaese install "Shapely" and copy my "hw2_chain_plotter.py" file to have a functioning code!
    import shapely # A Python library for set-theoretic analysis and manipulation of planar features

    Cobs = [] # The only output of this function which is the space of obstacle configurations
    obstacles = [shapely.Polygon(O[i]) for i in range(len(O))] # Obstacles are all polygons

    # Iterating over all possible configurations in the problem world
    for theta_1 in range(-180, 180, 1):
        for theta_2 in range(-180, 180, 1):
            config = [theta_1*numpy.pi/180, theta_2*numpy.pi/180]
            (joint_positions, link_vertices) = get_link_positions(config, W, L, D) # Transforming the robot (all linkages) according to the current config
            linkages = [shapely.Polygon(link_vertices[i]) for i in range(len(link_vertices))] # Converting the robot linkages into "Shapely" polygons for intersection check
            # Check if there is any intersections between any linkages and any obstacles 
            found_intersection = False
            for linkage in linkages:
                for obstacle in obstacles:
                    if linkage.intersects(obstacle):
                        Cobs.append((theta_1, theta_2))
                        found_intersection = True
                        break
                if found_intersection:
                    break
    return Cobs
    # raise NotImplementedError


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


class distance_computator(object):
    """A general class to measure the distance between two configurations"""
    def __init__(self, edge_type, step_size, min_turning_radius):
        self.edge_type = edge_type
        self.step_size = step_size
        self.min_turning_radius = min_turning_radius
        self.curvature = 1/min_turning_radius

    def get_distance(self, q1, q2):
        if self.edge_type == 'dubins':
            start_x, start_y, start_yaw = q1
            end_x, end_y, end_yaw = q2
            curvature = self.curvature
            path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
            return sum(lengths)
        elif self.edge_type == 'straight':
            return get_euclidean_distance(q1, q2)
        
    def get_nearest_on_edge(self, q1, q2, alpha_i):
        if self.edge_type == 'dubins':
            start_x, start_y, start_yaw = q1
            end_x, end_y, end_yaw = q2
            curvature = self.curvature
            path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
            min_distance = math.inf
            qn = q1
            for i in range(len(path_x)):
                path_x2, path_y2, path_yaw2, mode2, lengths2 = plan_dubins_path(path_x[i], path_y[i], path_yaw[i], alpha_i[0], alpha_i[1], alpha_i[2], curvature)
                if sum(lengths2) < min_distance:
                    min_distance = sum(lengths2)
                    qn = [path_x2[0], path_y2[0], path_yaw2[0]]
            return (tuple(qn), min_distance)
        elif self.edge_type == 'straight':
            return get_nearest_point_on_line(q1, q2, alpha_i, self.step_size)
        
    def get_local_path(self, q1, q2):
        if self.edge_type == 'dubins':
            start_x, start_y, start_yaw = q1
            end_x, end_y, end_yaw = q2
            curvature = self.curvature
            path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
            return (path_x, path_y, path_yaw)
        elif self.edge_type == 'straight':
            return None
            


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


class rectangular_obstacle(obstacle):
    """The rectangle subclass of "obstacle" """
    def __init__(self, vertices, W, L, D):
        x, y = 0, 0
        width, height = 0, 0
        self.width = width
        self.height = height
        for i in range(len(vertices)):
            x = x + vertices[i][0]
            y = y + vertices[i][1]
            width = abs(vertices[i][0] - vertices[0][0])
            height = abs(vertices[i][1] - vertices[0][1])
            if width > self.width:
                self.width = width
            if height > self.height:
                self.height = height
        self.origin = [x / (len(vertices)), y / (len(vertices))] # The center of the rectangle
        self.shape = shapely.Polygon(vertices) # Getting the Shapely object associated with the given rectangle for intersection/collision checks
        self.W, self.L, self.D = W, L, D # Storing the linkage parameters to use them in the "get_link_positions" function call (for the collision checks in the "contain" method)
    
    def get_bounds(self): # Not used in the current project as we do not intend to calculate C_obs explicitely
        return [(self.origin[0]-self.width/2, self.origin[0]+self.width/2), (self.origin[0]-self.height/2, self.origin[0]+self.height/2)]
    
    def get_boundary(self): # Not used in the current project as we do not intend to calculate C_obs explicitely
        return [
                [self.origin[0] - self.width/2, self.origin[0] - self.height/2],
                [self.origin[0] + self.width/2, self.origin[0] - self.height/2],
                [self.origin[0] + self.width/2, self.origin[0] + self.height/2],
                [self.origin[0] - self.width/2, self.origin[0] + self.height/2],
                [self.origin[0] - self.width/2, self.origin[0] - self.height/2]
        ]

    def contain(self, q):
        (joint_positions, link_vertices) = get_link_positions(q, self.W, self.L, self.D) # Transforming the robot (all linkages with paramters W, L, D) according to the current config "q"
        linkages = [shapely.Polygon(link_vertices[i]) for i in range(len(link_vertices))] # Converting the robot linkages into "Shapely" polygons for intersection check
        # Check if there is any intersections between any linkages and the obstacle object
        found_intersection = False
        for linkage in linkages:
            if linkage.intersects(self.shape):
                found_intersection = True
                break
        return found_intersection


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

