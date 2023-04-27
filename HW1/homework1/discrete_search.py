import math
from queue import Queue, QueueBFS, QueueDFS, QueueAstar

ALG_BFS = "bfs"
ALG_DFS = "dfs"
ALG_ASTAR = "astar"


class StateSpace:
    """A base class to specify a state space X"""

    def __contains__(self, x):
        """Return whether the given state x is in the state space"""
        raise NotImplementedError

    def get_distance_lower_bound(self, x1, x2):
        """Return the lower bound on the distance between the given states x1 and x2"""
        return 0


class ActionSpace:
    """A base class to specify an action space"""

    def __call__(self, x):
        """ Return the list of all the possible actions at the given state x"""
        raise NotImplementedError


class StateTransition:
    """A base class to specify a state transition function"""

    def __call__(self, x, u):
        """Return the new state obtained by applying action u at state x"""
        raise NotImplementedError


def fsearch(X, U, f, xI, XG, alg):
    """Return the list of visited nodes and a path from xI to XG based on the given algorithm

    This is the general template for forward search describe in Figure 2.4 in the textbook.

    @type X:   an instance of the StateSpace class (or its derived class) that represent the state space
    @type U:   an instance of the ActionSpace class (or its derived class) such that
               U(x) returns the list of all the possible actions at state x
    @type f:   an instance of the StateTransition class  (or its derived class) such that
               f(x, u) returns the new state obtained by applying action u at cell x
    @type xI:  an initial state such that xI in X
    @type XG:  a list of states that specify the goal set
    @type alg: a string in {"bfs", "dfs", "astar"} that specifies the discrete search algorithm to use

    @return:   a dictionary {"visited": visited_states, "path": path} where visited_states is the list of
               states visited during the search and path is a path from xI to a state in XG
    """
    if not (xI in X):
        return {"visited": [], "path": []}

    Q = get_queue(alg, X, XG)
    Q.insert(xI, None)

    while len(Q) > 0:
        x = Q.pop()
        if x in XG:
            return {"visited": Q.get_visited(), "path": Q.get_path(xI, x)}
        for u in U(x):
            xp = f(x, u)
            Q.insert(xp, x)

    return {"visited": Q.get_visited(), "path": []}


def get_queue(alg, X, XG):
    """Return the corresponding queue for a given algorithm

    @type alg: a string in {"bfs", "dfs", "astar"} that specifies the discrete search algorithm to use
    @type X:   an instance of the StateSpace class (or its derived class) that represent the state space
    @type XG:  a list of states that specify the goal set
    """

    class CostToGoEstimator:
        def __init__(self, X, XG):
            self.X = X
            self.XG = XG

        def get_lower_bound(self, x):
            """Return the lower bound on the distance between the given state x and the goal set XG"""
            min_cost = math.inf
            for xG in XG:
                cost = self.X.get_distance_lower_bound(x, xG)
                min_cost = min(min_cost, cost)
            return min_cost

    if alg == ALG_BFS:
        return QueueBFS()
    elif alg == ALG_DFS:
        return QueueDFS()
    elif alg == ALG_ASTAR:
        return QueueAstar(CostToGoEstimator(X, XG))
    return Queue()
