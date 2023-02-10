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
    # TODO: Implement this function
    print(alg)
    Q = get_queue(alg, X, XG)
    cost_to_goal = abs(X.Xmax - X.Xmin) + abs(X.Ymax - X.Ymin)
    for xG in XG:
        temp = X.get_distance_lower_bound(xI, xG)
        if temp < cost_to_goal:
            cost_to_goal = temp
    node = Node(xI, None, 0, cost_to_goal)
    Q.insert(node)
    dic = {"visited": [], "path": []}

    iteration = 0
    while len(Q.Q) != 0:
        node = Q.pop()
        cost_so_far = node.f
        dic["visited"].append(node.x)
        if node.x in XG:
            dic["path"] = Q.path(node.x)
            return dic
        for u in U(node.x):
            x_new = f(node.x, u)
            node.parent = node.x
            node.x = x_new
            node.f = cost_so_far + 1
            if x_new not in dic["visited"]:
                Q.insert(node)
        iteration = iteration + 1
    
    return "FAILURE"
    raise NotImplementedError


from collections import deque
def get_queue(alg, X, XG):
    if alg == "bfs":
        Q = QueueBFS()
    elif alg == "dfs":
        Q = QueueDFS()
    else:
        Q = QueueAstar(X, XG)
    return Q


class Node():
    def __init__(self, x, parent, f, cost_total):
        self.x = x
        self.parent = parent
        self.f = f
        self.cost_total = cost_total


class Queue():
    """A base class of queue data structure"""

    def __init__(self):
        """Return whether the given state x is in the state space"""
        self.Q = deque()

    def pop(self):
        return self.Q.popleft()
    
    def insert(self, node):
        raise NotImplementedError
    
    def path(self, node):
        path_back = [node.x]
        while node.parent:
            path_back.append(node.parent)
            node = node.parent
        return list(reversed(path_back))

    

class QueueBFS(Queue):
    """A derivative class of Queue for BFS algorithm"""

    def __init__(self):
        super().__init__()

    def pop(self):
        return self.Q.popleft()
    
    def insert(self, node):
        found_state_in_queue = False
        for i in range(len(self.Q)):
            if node.x == self.Q[i].x:
                found_state_in_queue = True
        if not found_state_in_queue: 
            self.Q.append(node)
    

class QueueDFS(Queue):
    """A derivative class of Queue for DFS algorithm"""

    def __init__(self):
        super().__init__()

    def pop(self):
        return self.Q.pop()
    
    def insert(self, node):
        found_state_in_queue = False
        for i in range(len(self.Q)):
            if node.x == self.Q[i].x:
                found_state_in_queue = True
        if not found_state_in_queue: 
            self.Q.append(node)


class QueueAstar(Queue):
    """A derivative class of Queue for Astar algorithm"""

    def __init__(self, X, XG):
        super().__init__()
        self.X = X
        self.XG = XG

    def pop(self):
        return self.Q.pop()
    
    def insert(self, node):
        for i in range(len(self.Q)):
            if node.x == self.Q[i].x:
                if node.f < self.Q[i].f:
                    del self.Q[i]
                    break
                else:
                    return
        
        cost_to_goal = abs(self.X.Xmax - self.X.Xmin) + abs(self.X.Ymax - self.X.Ymin)
        for xG in self.XG:
            temp = self.X.get_distance_lower_bound(node.x, xG)
            if temp < cost_to_goal:
                cost_to_goal = temp
        node.cost_total = cost_to_goal + node.f
        
        i = 0
        for i in range(1, len(self.Q) + 1):
            if node.cost_total < self.Q[-i].cost_total:
                break
        self.Q.insert(-i, node)

