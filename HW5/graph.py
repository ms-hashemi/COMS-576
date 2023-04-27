# The following classes are imported from "discrete_search.py" (from the first class assignment) 
# to implement/subclass its base classes such that they are consistent with the data structures and the 
# the specific algorithmic implementation of the current assignment. This was only needed for 
# the graph search (like BFS) to find possible paths between qI and qG in the PRM algorithm, 
# not the RRT one, as RRT results in a simple connected tree where paths can be found easily based on
# my current implementation of the graph data strcuture (which is a tree when it is used in RRT 
# and a graph when it is used in PRM)
from discrete_search import (
    StateSpace,
    ActionSpace,
    StateTransition
)
import math


class Graph(object):
    """A class defined to implement graph/tree data structure"""

    def __init__(self, if_tree):
        """Initializing the graph by defining the object properties"""
        self.adjacency = []
        self.vertices = []
        self.edges = []
        self.number_vertices = 0
        self.if_tree = if_tree

    def add_vertex(self, vertex):
        """
        Adds a vertex to the graph
        """
        self.adjacency.append([])
        self.vertices.append(vertex)
        self.number_vertices = self.number_vertices + 1

    def add_edge(self, vertex, parent):
        """
        Adds a new edge such that the "vertex" is connected to its "parent" vertex in the graph
        """
        if vertex["id"] < len(self.adjacency):
            if self.if_tree:
                self.adjacency[vertex["id"]].insert(0, parent["id"])
                self.adjacency[parent["id"]].append(vertex["id"])
            else:
                self.adjacency[parent["id"]].append(vertex["id"])
            self.edges.append([parent["id"], vertex["id"]])
    
    def same_component(self, alpha_i, q, k):
        """
        Note: this is not exactly a "same_component" function in the PRM algorithm decription; it is 
        an alternative function suggested in the reference book of the class. It considers the degree
        of the "q" vertex to determine whether the new vertex "alpha_i" should be connected to "q" or not.
        """
        return not(len(self.adjacency[q["id"]]) < k and len(self.adjacency[alpha_i["id"]]) < k) # and alpha_i["id"] not in self.adjacency[q["id"]]


class Grid2DStates(StateSpace):
    def __init__(self, graph, d, Xmin, Xmax, Ymin, Ymax, O):
        """
        Xmin, Xmax, Ymin, Ymax: floats that defines the boundary of the world
        O:    a list of tuples that represent the grid points
              that are occupied by an obstacle.
              A tuple (i, j) in O means that grid point (i,j) is occupied.
        """
        self.graph = graph
        self.d = d
        self.Xmin = Xmin
        self.Xmax = Xmax
        self.Ymin = Ymin
        self.Ymax = Ymax
        self.O = O

    def __contains__(self, x):
        # TODO: Implement this function
        if self.Xmin <= x[0] and x[0] <= self.Xmax and self.Ymin <= x[1] and x[1] <= self.Ymax and x not in self.O:
            return True
        else:
            return False
        raise NotImplementedError

    def get_distance_lower_bound(self, x1, x2):
        # TODO: Implement this function
        return self.d.get_distance(self.graph.vertices[x1]["config"], self.graph.vertices[x2]["config"])
        raise NotImplementedError

    def draw(self, ax):
        """A function used to draw the graph. It is called inside "draw_cspace.py" """
        # arrow_length = 0.1
        # head_width = 0.1
        # fc="k"
        # ec="k"
        if len(self.graph.vertices) > 0:
            for edge in self.graph.edges:
                parent = self.graph.vertices[edge[0]]["config"]
                vertex = self.graph.vertices[edge[1]]["config"]
                path_x, path_y, path_yaw = self.d.get_local_path(parent, vertex)
                ax.plot(path_x, path_y, color='black')
                ax.plot(parent[0], parent[1], marker=(3, 0, parent[2] * 180 / math.pi - 90), markersize=15, linestyle="None", markerfacecolor="black", markeredgecolor="black")
                ax.plot(vertex[0], vertex[1], marker=(3, 0, vertex[2] * 180 / math.pi - 90), markersize=15, linestyle="None", markerfacecolor="black", markeredgecolor="black")
                # x, y, yaw = path_x[0], path_y[0], path_yaw[0]
                # ax.arrow(x, y, arrow_length * math.cos(yaw), arrow_length * math.sin(yaw), head_width=head_width, fc=fc, ec=ec)
                # x, y, yaw = path_x[-1], path_y[-1], path_yaw[-1]
                # ax.arrow(x, y, arrow_length * math.cos(yaw), arrow_length * math.sin(yaw), head_width=head_width, fc=fc, ec=ec)


class GridStateTransition(StateTransition):
    def __init__(self, graph):
        self.graph = graph
    
    def __call__(self, x, u):
        # TODO: Implement this function
        return self.graph.adjacency[x][u]
        raise NotImplementedError


class Grid2DActions(ActionSpace):
    def __init__(self, X, f):
        self.X = X
        self.f = f

    def __call__(self, x):
        # TODO: Implement this function
        possible_actions = [u for u in range(len(self.f.graph.adjacency[x]))]
        return possible_actions
        raise NotImplementedError