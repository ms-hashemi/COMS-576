import random
import numpy as np
from graph import Tree, GraphCC
from edge import EdgeStraight
from geometry import get_euclidean_distance


##############################################################################
# Classes for creating an edge
##############################################################################
class EdgeCreator:
    def make_edge(self, s1, s2):
        """Return an Edge object beginning at state s1 and ending at state s2"""
        raise NotImplementedError


class StraightEdgeCreator(EdgeCreator):
    def __init__(self, step_size):
        self.step_size = step_size

    def make_edge(self, s1, s2):
        return EdgeStraight(s1, s2, self.step_size)


##############################################################################
# Classes for computing distance between 2 points
##############################################################################
class DistanceComputator:
    def get_distance(self, s1, s2):
        """Return the distance between s1 and s2"""
        raise NotImplementedError


class EuclideanDistanceComputator(DistanceComputator):
    def get_distance(self, s1, s2):
        """Return the Euclidean distance between s1 and s2"""
        return get_euclidean_distance(s1, s2)


##############################################################################
# Classes for collision checking
##############################################################################
class CollisionChecker:
    def is_in_collision(self, state):
        """Return whether the given state is in collision"""
        raise NotImplementedError

    def is_checking_required(self):
        """Return whether collision needs to be checked at all"""
        raise NotImplementedError


class EmptyCollisionChecker(CollisionChecker):
    def is_in_collision(self, state):
        """Return whether the given state is in collision"""
        return False

    def is_checking_required(self):
        """Return whether collision needs to be checked at all"""
        return False


class ObstacleCollisionChecker(CollisionChecker):
    def __init__(self, obstacles):
        """The constructor

        @type obstacles: a list [obs_1, ..., obs_m] of obstacles, where obs_i is an Obstacle
            object that include a contain(s) function, which returns whether a state s
            is inside the obstacle
        """
        self.obstacles = obstacles

    def is_in_collision(self, s):
        """Return whether the point s is in collision with the obstacles"""
        for obs in self.obstacles:
            if obs.contain(s):
                return True
        return False

    def is_checking_required(self):
        """Return whether collision needs to be checked at all"""
        return True


##############################################################################
# Planning algorithms
##############################################################################
def rrt(
    cspace,
    qI,
    qG,
    edge_creator,
    distance_computator,
    collision_checker,
    pG=0.1,
    numIt=100,
    tol=1e-3,
):
    """RRT with obstacles

    @type cspace: a list of tuples (smin, smax) indicating that the C-space
        is given by the product of the tuples.
    @type qI: a tuple (x, y) indicating the initial configuration.
    @type qG: a typle (x, y) indicating the goal configuation
        (can be None if rrt is only used to explore the C-space).
    @type edge_creator: an EdgeCreator object that includes the make_edge(s1, s2) function,
        which returns an Edge object beginning at state s1 and ending at state s2.
    @type distance_computator: a DistanceComputator object that includes the get_distance(s1, s2)
        function, which returns the distance between s1 and s2.
    @type collision_checker: a CollisionChecker object that includes the is_in_collision(s)
        function, which returns whether the state s is in collision.
    @type pG: a float indicating the probability of choosing the goal configuration.
    @type numIt: an integer indicating the maximum number of iterations.
    @type tol: a float, indicating the tolerance on the euclidean distance when checking whether
        2 states are the same

    @return (G, root, goal) where G is the tree, root is the id of the root vertex
        and goal is the id of the goal vertex (if one exists in the tree; otherwise goal will be None).
    """
    G = Tree()
    root = G.add_vertex(np.array(qI))
    for i in range(numIt):
        use_goal = qG is not None and random.uniform(0, 1) <= pG
        if use_goal:
            alpha = np.array(qG)
        else:
            alpha = sample(cspace)
        vn = G.get_nearest(alpha, distance_computator, tol)
        qn = G.get_vertex_state(vn)
        (qs, edge) = stopping_configuration(
            qn, alpha, edge_creator, collision_checker, tol
        )
        if qs is None or edge is None:
            continue
        dist = get_euclidean_distance(qn, qs)
        if dist > tol:
            vs = G.add_vertex(qs)
            G.add_edge(vn, vs, edge)
            if use_goal and get_euclidean_distance(qs, qG) < tol:
                return (G, root, vs)

    return (G, root, None)


def rrt_optimal(
    cspace,
    qI,
    qG,
    edge_creator,
    distance_computator,
    collision_checker,
    pG=0.1,
    numIt=100,
    tol=1e-3,
):
    """RRT with obstacles

    @type cspace: a list of tuples (smin, smax) indicating that the C-space
        is given by the product of the tuples.
    @type qI: a tuple (x, y) indicating the initial configuration.
    @type qG: a typle (x, y) indicating the goal configuation
        (can be None if rrt is only used to explore the C-space).
    @type edge_creator: an EdgeCreator object that includes the make_edge(s1, s2) function,
        which returns an Edge object beginning at state s1 and ending at state s2.
    @type distance_computator: a DistanceComputator object that includes the get_distance(s1, s2)
        function, which returns the distance between s1 and s2.
    @type collision_checker: a CollisionChecker object that includes the is_in_collision(s)
        function, which returns whether the state s is in collision.
    @type pG: a float indicating the probability of choosing the goal configuration.
    @type numIt: an integer indicating the maximum number of iterations.
    @type tol: a float, indicating the tolerance on the euclidean distance when checking whether
        2 states are the same

    @return (G, root, goal) where G is the tree, root is the id of the root vertex
        and goal is the id of the goal vertex (if one exists in the tree; otherwise goal will be None).
    """
    import copy
    G = Tree()
    costs = {}
    root = G.add_vertex(np.array(qI))
    costs[root] = 0
    for i in range(numIt):
        use_goal = qG is not None and random.uniform(0, 1) <= pG
        if use_goal:
            alpha = np.array(qG)
        else:
            alpha = sample(cspace)
        vn = G.get_nearest(alpha, distance_computator, tol)
        qn = G.get_vertex_state(vn)
        if i > 0 and vn not in costs:
            costs[vn] = costs[G.parents[vn][0]] + G.edges[(G.parents[vn][0], vn)][0]
        (qs, edge) = stopping_configuration(
            qn, alpha, edge_creator, collision_checker, tol
        )
        if qs is None or edge is None:
            continue
        dist = get_euclidean_distance(qn, qs)
        if dist > tol:
            if len(G.vertices) <= 1:
                k = len(G.vertices)
            else:
                k = int(np.ceil((2*np.exp(1)+0.001)*np.log(len(G.vertices))))
            neighbors = G.get_nearest_vertices(qs, k, distance_computator)
            vs = G.add_vertex(qs)
            v_min = copy.deepcopy(vn); q_min = copy.deepcopy(qn); c_min = costs[vn] + edge.get_cost()
            
            # Connect along a minimum-cost path
            for v_n in neighbors:
                q_n = G.get_vertex_state(v_n)
                edge1 = edge_creator.make_edge(q_n, qs)
                cost = costs[v_n] + edge1.get_cost()
                if connect(q_n, qs, edge_creator, collision_checker, tol) and cost < c_min:
                    v_min = copy.deepcopy(v_n); q_min = copy.deepcopy(q_n); c_min = copy.deepcopy(cost)
            G.add_edge(v_min, vs, edge_creator.make_edge(q_min, qs))
            costs[vs] = c_min

            if use_goal and get_euclidean_distance(qs, qG) < tol:
                return (G, root, vs)
            
            # Rewire the tree
            for v_n in neighbors:
                q_n = G.get_vertex_state(v_n)
                edge1 = edge_creator.make_edge(qs, q_n)
                cost = costs[vs] + edge1.get_cost()
                if connect(qs, q_n, edge_creator, collision_checker, tol) and cost < costs[v_n]:
                    G.remove_edge((G.parents[v_n][0], v_n))
                    G.add_edge(vs, v_n, edge_creator.make_edge(qs, q_n))
                    costs[v_n] = cost

    return (G, root, None)


def prm(
    cspace,
    qI,
    qG,
    edge_creator,
    distance_computator,
    collision_checker,
    k,
    numIt=1000,
    tol=1e-3,
):
    """PRM with obstacles

    @type cspace: a list of tuples (smin, smax) indicating that the C-space
        is given by the product of the tuples.
    @type qI: a tuple (x, y) indicating the initial configuration.
    @type qG: a typle (x, y) indicating the goal configuation
        (can be None if prm is only used to explore the C-space).
    @type edge_creator: an EdgeCreator object that includes the make_edge(s1, s2) function,
        which returns an Edge object beginning at state s1 and ending at state s2.
    @type distance_computator: a DistanceComputator object that includes the get_distance(s1, s2)
        function, which returns the distance between s1 and s2.
    @type collision_checker: a CollisionChecker object that includes the is_in_collision(s)
        function, which returns whether the state s is in collision.
    @type k: a float, indicating the number of nearest neighbors

    @return (G, root, goal) where G is the roadmap, root is the id of the root vertex
        and goal is the id of the goal vertex.
        If the root (resp. goal) vertex does not exist in the roadmap, root (resp. goal) will be None.
    """

    def add_to_roadmap(G, alpha):
        """Add configuration alpha to the roadmap G"""
        if collision_checker.is_in_collision(alpha):
            return None
        neighbors = G.get_nearest_vertices(alpha, k, distance_computator)
        vs = G.add_vertex(alpha)
        for vn in neighbors:
            if G.is_same_component(vn, vs):
                continue
            qn = G.get_vertex_state(vn)
            if connect(alpha, qn, edge_creator, collision_checker, tol) and connect(
                qn, alpha, edge_creator, collision_checker, tol
            ):
                G.add_edge(vs, vn, edge_creator.make_edge(alpha, qn))
        return vs

    G = GraphCC()
    i = 0
    while i < numIt:
        alpha = sample(cspace)
        if add_to_roadmap(G, alpha) is not None:
            i = i + 1
    root = None
    if qI is not None:
        root = add_to_roadmap(G, np.array(qI))
    goal = None
    if qG is not None:
        goal = add_to_roadmap(G, np.array(qG))
    return (G, root, goal)


def prm_optimal(
    cspace,
    qI,
    qG,
    edge_creator,
    distance_computator,
    collision_checker,
    k,
    numIt=1000,
    tol=1e-3,
):
    """PRM with obstacles

    @type cspace: a list of tuples (smin, smax) indicating that the C-space
        is given by the product of the tuples.
    @type qI: a tuple (x, y) indicating the initial configuration.
    @type qG: a typle (x, y) indicating the goal configuation
        (can be None if prm is only used to explore the C-space).
    @type edge_creator: an EdgeCreator object that includes the make_edge(s1, s2) function,
        which returns an Edge object beginning at state s1 and ending at state s2.
    @type distance_computator: a DistanceComputator object that includes the get_distance(s1, s2)
        function, which returns the distance between s1 and s2.
    @type collision_checker: a CollisionChecker object that includes the is_in_collision(s)
        function, which returns whether the state s is in collision.
    @type k: a float, indicating the number of nearest neighbors

    @return (G, root, goal) where G is the roadmap, root is the id of the root vertex
        and goal is the id of the goal vertex.
        If the root (resp. goal) vertex does not exist in the roadmap, root (resp. goal) will be None.
    """

    def add_to_roadmap(G, alpha):
        """Add configuration alpha to the roadmap G"""
        if collision_checker.is_in_collision(alpha):
            return None
        if len(G.vertices) <= 1:
            k = len(G.vertices)
        else:
            k = int(np.ceil((2*np.exp(1)+0.001)*np.log(len(G.vertices))))
        neighbors = G.get_nearest_vertices(alpha, k, distance_computator)
        vs = G.add_vertex(alpha)
        for vn in neighbors:
            if G.is_same_component(vn, vs):
                continue
            qn = G.get_vertex_state(vn)
            if connect(alpha, qn, edge_creator, collision_checker, tol) and connect(
                qn, alpha, edge_creator, collision_checker, tol
            ):
                G.add_edge(vs, vn, edge_creator.make_edge(alpha, qn))
        return vs

    G = GraphCC()
    i = 0
    while i < numIt:
        alpha = sample(cspace)
        if add_to_roadmap(G, alpha) is not None:
            i = i + 1
    root = None
    if qI is not None:
        root = add_to_roadmap(G, np.array(qI))
    goal = None
    if qG is not None:
        goal = add_to_roadmap(G, np.array(qG))
    return (G, root, goal)


def sample(cspace):
    """Return a sample configuration of the C-space based on uniform random sampling"""
    sample = [random.uniform(cspace_comp[0], cspace_comp[1]) for cspace_comp in cspace]
    return np.array(sample)


def stopping_configuration(s1, s2, edge_creator, collision_checker, tol):
    """Return (s, edge) where s is the point along the edge from s1 to s2 that is closest to s2 and
    is not in collision with the obstacles and edge is the edge from s to s1"""

    edge = edge_creator.make_edge(s1, s2)
    if not collision_checker.is_checking_required():
        return (s2, edge)

    if edge.get_length() < tol:
        return (s1, edge)

    curr_ind = 0
    prev_state = None
    curr_state = edge.get_discretized_state(curr_ind)

    while curr_state is not None:
        if collision_checker.is_in_collision(curr_state):
            if curr_ind == 0:
                return (None, None)
            elif curr_ind == 1:
                return (s1, None)
            split_t = (curr_ind - 1) * edge.get_step_size() / edge.get_length()
            (edge1, _) = edge.split(split_t)
            return (prev_state, edge1)
        curr_ind = curr_ind + 1
        prev_state = curr_state
        curr_state = edge.get_discretized_state(curr_ind)

    return (s2, edge)


def connect(s1, s2, edge_creator, collision_checker, tol):
    """Return whether an edge between s1 and s2 is collision-free"""
    if not collision_checker.is_checking_required():
        return True

    edge = edge_creator.make_edge(s1, s2)
    if edge.get_length() < tol:
        return True

    curr_ind = 0
    curr_state = edge.get_discretized_state(curr_ind)
    while curr_state is not None:
        if collision_checker.is_in_collision(curr_state):
            return False
        curr_ind = curr_ind + 1
        curr_state = edge.get_discretized_state(curr_ind)

    return True
