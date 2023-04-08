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


class Graph(object):
    """A class defined to implement graph/tree data structure"""

    def __init__(self):
        """Initializing the graph by defining the object properties"""
        self.adjacency = []
        self.vertices = []
        self.edges = []
        self.number_vertices = 0

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
            self.adjacency[vertex["id"]].insert(0, parent["id"])
            self.adjacency[parent["id"]].append(vertex["id"])
            self.edges.append([vertex["id"], parent["id"]])
    
    def same_component(self, alpha_i, q, k):
        """
        Note: this is not exactly a "same_component" function in the PRM algorithm decription; it is 
        an alternative function suggested in the reference book of the class. It considers the degree
        of the "q" vertex to determine whether the new vertex "alpha_i" should be connected to "q" or not.
        """
        return not(len(self.adjacency[q]) < k)


class Grid2DStates(StateSpace):
    def __init__(self, graph, Xmin, Xmax, Ymin, Ymax, O):
        """
        Xmin, Xmax, Ymin, Ymax: floats that defines the boundary of the world
        O:    a list of tuples that represent the grid points
              that are occupied by an obstacle.
              A tuple (i, j) in O means that grid point (i,j) is occupied.
        """
        self.graph = graph
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
        return ((x1[0] - x2[0])**2 + (x1[1] - x2[1])**2)**0.5
        raise NotImplementedError

    def draw(self, ax):
        """A function used to draw the graph. It is called inside "draw_cspace.py" """
        if len(self.graph.vertices) > 0:
            for edge in self.graph.edges:
                vertex = self.graph.vertices[edge[0]]["config"]
                parent = self.graph.vertices[edge[1]]["config"]
                ax.plot([vertex[0], parent[0]], [vertex[1], parent[1]], color='black')


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