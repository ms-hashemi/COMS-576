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

import copy
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
    # Initialization and calculating the minimum expected cost to a goal from the initial state
    Q = get_queue(alg, X, XG)
    cost_to_goal = abs(X.Xmax - X.Xmin) + abs(X.Ymax - X.Ymin) # Maximum possible cost in the 2D grid
    for xG in XG:
        temp = X.get_distance_lower_bound(xI, xG)
        if temp < cost_to_goal:
            cost_to_goal = temp
    # Initial node
    node = Node(xI, None, 0, cost_to_goal)
    # A blank node for children
    node_child = Node(None, None, None, cost_to_goal)
    # Inserting the initial node for the initial state
    Q.insert(node)
    # The result of this function as a dictionary
    dic = {"visited": [], "path": []}
    dic["visited"].append(node.x)


    # The following is basically the implementation of the general FORWARD_SEARCH algorithm of the book
    iteration = 0
    while len(Q.Q) != 0:
        node = copy.deepcopy(Q.pop())
        cost_so_far = node.f
        # print("current state:", node.x)
        if node.x in XG:
            # if node.x not in dic["visited"]:
                # dic["visited"].append(node_child.x)
            dic["path"] = Q.path(node)
            return dic
        for u in U(node.x):
            # print("x:", node.x)
            # print("u:", u)
            x_new = f(node.x, u)
            node_child.parent = node
            node_child.x = x_new
            node_child.f = cost_so_far + 1
            # print("length of queue:", len(Q.Q))
            if x_new not in dic["visited"]:
                Q.insert(node_child)
                dic["visited"].append(node_child.x)
                # print("new length of queue:", len(Q.Q))
        iteration = iteration + 1
    
    # return "FAILURE"
    return dic
    raise NotImplementedError


# deque object from collections was imported becuase it has 
# a better performance than a simple python list object in BFS
from collections import deque

def get_queue(alg, X, XG):
    """A method which returns the appropriate queue data structure for the given algorithm "alg" """
    if alg == "bfs":
        Q = QueueBFS()
    elif alg == "dfs":
        Q = QueueDFS()
    else:
        # X and XG are needed for the ASTAR algorithm as explained in the assignment
        Q = QueueAstar(X, XG)
    return Q


class Node():
    """A class defining node objects for graph search problems"""
    def __init__(self, x, parent, f, cost_total):
        self.x = x
        self.parent = parent
        # Cost so far (up to this node): f
        self.f = f
        # cost_total = f + minimum_expected_remaining_cost_to_a_goal
        # This will be calculated for the ASTAR algorithm in the "insert" method of "QueueAstar" object below
        self.cost_total = cost_total

    # A method to compare two nodes ("self" and "other") according to their "cost_total" attribute
    # It will be implicitely used in the ASTAR algorithm ("QueueAstar" object below) to sort the queue in a descending order
    def __lt__(self, other):
        return self.cost_total < other.cost_total


class Queue():
    """A base class of queue data structure"""

    def __init__(self):
        """The class has one attribute, the queue denoted by Q"""
        # deque object from collections was imported becuase it has 
        # a better performance than a simple python list class in BFS
        self.Q = deque()

    def pop(self):
        """A method for popping or removing an element/object from the queue"""
        # The default behavior is popping from the left (first element)
        return self.Q.popleft()
    
    def insert(self, node):
        """A method for inserting an element/object into the queue"""
        # The main difference between the search algorithms/data structures defined below
        self.Q.append(copy.deepcopy(node))
    
    def path(self, node):
        """A method generating a path from the initial state to the current node state"""
        path_back = []
        while node:
            path_back.append(node.x)
            node = node.parent
        return list(reversed(path_back)) # The "path_back" should be reversed to show the path from the initial node to the current state/goal
    

class QueueBFS(Queue):
    """A derivative class of Queue for BFS algorithm"""

    def __init__(self):
        super().__init__()

    def pop(self):
        return self.Q.popleft() # due to the need for a FIFO queue in BFS algorithm
    
    def insert(self, node):
        # Check whether a similar state has already been found or exist in the queue to avoid loops in search through duplicate states
        found_state_in_queue = False
        for i in range(len(self.Q)):
            if node.x == self.Q[i].x:
                found_state_in_queue = True
                break
        if not found_state_in_queue: 
            self.Q.append(copy.deepcopy(node)) # due to the need for a FIFO queue in the BFS algorithm
    

class QueueDFS(Queue):
    """A derivative class of Queue for DFS algorithm"""

    def __init__(self):
        super().__init__()

    def pop(self):
        return self.Q.pop() # due to the need for a LIFO queue in the DFS algorithm
    
    def insert(self, node):
        # Check whether a similar state has already been found or exist in the queue to avoid loops in search through duplicate states
        found_state_in_queue = False
        for i in range(len(self.Q)):
            if node.x == self.Q[i].x:
                found_state_in_queue = True
                break
        if not found_state_in_queue: 
            self.Q.append(copy.deepcopy(node)) # due to the need for a LIFO queue in the DFS algorithm


class QueueAstar(Queue):
    """A derivative class of Queue for Astar algorithm"""

    def __init__(self, X, XG):
        super().__init__()
        self.X = X
        self.XG = XG

    def pop(self):
        return self.Q.pop() # due to the need for a PRIORITY queue/MIN HEAP in the ASTAR algorithm
    
    def insert(self, node):
        # Removing the nodes with the exact current states if their "f" is greater than that of the input "node"
        # If their "f" is lower, then the input "node" will NOT be INSERTED into the queue
        for i in range(len(self.Q)):
            if node.x == self.Q[i].x:
                if node.f < self.Q[i].f:
                    del self.Q[i]
                    break
                else:
                    return
        
        # Calculating the minimum total expected cost to a goal state from the current node
        cost_to_goal = abs(self.X.Xmax - self.X.Xmin) + abs(self.X.Ymax - self.X.Ymin)
        for xG in self.XG:
            temp = self.X.get_distance_lower_bound(node.x, xG)
            if temp < cost_to_goal:
                cost_to_goal = temp
        node.cost_total = cost_to_goal + node.f
        
        # for i in range(1, len(self.Q) + 1):
        #     if node.cost_total < self.Q[-i].cost_total:
        #         break
        # self.Q.insert(-i, copy.deepcopy(node))

        # Inserting the node simply at the end of the queue
        self.Q.append(copy.deepcopy(node))
        # Sorting the queue in a descending order according to its nodes' minimum total expected cost
        self.Q = sorted(self.Q, reverse=True)

        # print("Queue:")
        # for i in range(len(self.Q)):
        #     print(self.Q[i].x, self.Q[i].cost_total)

