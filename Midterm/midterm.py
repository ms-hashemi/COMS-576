#############################################################################################################
# COM S 576 Midterm Problem 1 Solution
# Mohammad Hashemi
#############################################################################################################

import json, sys, os, argparse # For parsing the program arguments and outputing the json results
import math # For PI and INF parameters
import numpy as np # For random samplings
import copy
import matplotlib.pyplot as plt # For plotting the results
from discrete_search import fsearch, ALG_BFS # For graph search (needed for the general graph search problem encountered in PPM algorithm); not used in the current project/midterm exam!
from draw_cspace import draw # For drawing the outputs ("draw_cspace" is the modified version of the same code given by the professor/in her Github)
import geometry # An auxiliary python module written by me for handling the geometrical objects and functions including definition of basic obstacle classes and the relavant methods to determine collision with them in different robot configurations
import graph # An auxiliary python module written by me for handling the graph objects and its related methods (graph is defined in a general manner such that it can be used for both single tree-based RRT and complex graph-based PRM)


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Run forward search")
    parser.add_argument(
        "desc",
        metavar="problem_description_path",
        type=str,
        help="path to the problem description file containing the obstacle region in the world as well as the size and shape of the robot, including the width and length of each link, and the distance between two points of attachment",
    )
    parser.add_argument(
        "--out",
        metavar="output_path",
        type=str,
        required=False,
        default="",
        dest="out",
        help="path to the output file",
    )

    args = parser.parse_args(sys.argv[1:])
    if not args.out:
        args.out = os.path.splitext(os.path.basename(args.desc))[0] + "_out" + ".json"

    print("Problem description: ", args.desc)
    print("Output:              ", args.out)

    return args


def parse_desc(desc):
    """Parse problem description json file to get the problem description"""
    with open(desc) as desc:
        data = json.load(desc)

    # The exception handling blocks are written such that this function can be used for both Project/HW4 and Midterm
    O = data["O"]
    try:
        dt = data["dt"]
    except:
        dt = 0
    try:
        ORIGIN = data["ORIGIN"]
    except:
        ORIGIN = [0, 0]
    W = data["W"]
    try:
        H = data["H"]
        L = 0
        D = 0
    except:
        H = 2 * math.pi
        L = data["L"]
        D = data["D"]
    try:
        QUERIES = data["QUERIES"]
    except:
        QUERIES = [[data["xI"], data["xG"]]]
    try:
        MAX_SAMPLES = data["MAX_SAMPLES"]
    except:
        MAX_SAMPLES = 500
    try:
        RESOLUTION = data["RESOLUTION"]
    except:
        RESOLUTION = 0.1
    try:
        PROBABILITY_CONNECT_GOAL = data["PROBABILITY_CONNECT_GOAL"]
    except:
        PROBABILITY_CONNECT_GOAL = 0.01
    try:
        K = data["K"]
    except:
        K = 15
    return (O, dt, ORIGIN, W, H, L, D, QUERIES, MAX_SAMPLES, RESOLUTION, PROBABILITY_CONNECT_GOAL, K)


class search_space(object):
    """A class to find feasible samples in the world"""
    def __init__(self, world, obstacles):
        """Initializing the class properties"""
        self.world = world
        self.obstacles = obstacles

    def empty_collision_checker(self, q):
        """Returns True if the vertex or state q is inside the world boundaries; otherwise, False"""
        return self.world.contain(q)
    
    def obstacle_free(self, q):
        """Returns True if the vertex or state q is not inside any obstacles; otherwise, False"""
        collision = False
        if self.obstacles:
            for obs in self.obstacles:
                if obs.contain(q):
                    collision = True
                    break
        return not collision

    def sample(self):
        """Returns a random (2D) sample inside the world boundaries"""
        bounds = np.array(self.world.get_bounds())
        return tuple(np.random.uniform(bounds[:, 0], bounds[:, 1]))

    def sample_free(self):
        """Loops until it finds a feasible/collision-free sample of the world"""
        while True:  # sample until not inside of an obstacle
            q = self.sample()
            if self.obstacle_free(q) and self.empty_collision_checker(q):
                return q


def stopping_config(s, q1_line, q2_line, delta_q):
    """Auxiliary function for RRT to find the point on the new edge before obstacle"""
    direction = []
    length = geometry.get_euclidean_distance(q1_line, q2_line)
    for i in range(len(q1_line)):
        direction.append((q2_line[i]-q1_line[i])/length)
    i = 0
    qs = list(copy.deepcopy(q1_line))
    qs_new = list(copy.deepcopy(q1_line))
    while length > 0:
        if length < delta_q and i > 0:
            for j in range(len(q2_line)):
                qs_new[j] = q2_line[j]
            if s.empty_collision_checker(qs_new) and s.obstacle_free(qs_new):
                return tuple(qs_new)
            else:
                return tuple(qs)
        else:
            for j in range(len(q1_line)):
                qs_new[j] = q1_line[j] + i * delta_q * direction[j]
            if s.empty_collision_checker(qs_new) and s.obstacle_free(qs_new):
                qs = copy.deepcopy(qs_new)
            else:
                return tuple(qs)
            length = length - delta_q
            i = i + 1
    return tuple(qs)


def RRT(world, obstacles, query, max_samples = 1000, resolution = 0.1, p = 0.01):
    """Main RRT function which generates an exploring tree and possibly a tree connecting qI to qG as
    defined by query (if query is empty, it just generates the tree)
    
    @returns: a tuple of the exploring tree "g" and the found "path" ("path" is empty if there is no
              query or the path could not be found given the maximum sampling iterations).
    """
    s = search_space(world, obstacles)
    g = graph.Graph()
    if query is None:
        qI = s.sample_free()
        qG = s.sample_free()
    else:
        qI = tuple(query[0])
        qG = tuple(query[1])
    g.add_vertex({"id": g.number_vertices, "config": qI})

    path = []
    if query is not None: # If there is a specific query or tuple of (qI, qG) as the target of the algorithm
        for i in range(max_samples):
            if i > 0 and i % 50 == 0: # Periodically try to connect to qG
                alpha_i = qG
            else: # Probablistically (1% by default) try to connect to qG
                if np.random.uniform(0, 1) > p:
                    alpha_i = s.sample_free()
                else:
                    alpha_i = qG
            min_distance = math.inf
            qn = copy.deepcopy(qI)
            vertex = {"id": 0, "config": qI}
            parent = {"id": 0, "config": qI}
            edge_index = 0
            for counter, edge in enumerate(g.edges): # Loop over the edges to find the closest point of the tree (qn) to alpha_i
                (q, distance) = geometry.get_nearest_point_on_line(g.vertices[edge[0]]["config"], g.vertices[edge[1]]["config"], alpha_i, resolution)
                if distance < min_distance:
                    min_distance = distance
                    qn = copy.deepcopy(q)
                    vertex = {"id": edge[0], "config": g.vertices[edge[0]]["config"]}
                    parent = {"id": edge[1], "config": g.vertices[edge[1]]["config"]}
                    edge_index = counter
            qs = stopping_config(s, qn, alpha_i, resolution) # Find the last feasible/obstacle-free config (qs) along an imaginary edge/line segment from qn towards alpha_i
            id_new = 0
            if qs != qn: # Check if qs is different than qn (if not, no change is needed in the tree)
                if qn != vertex["config"] and qn != parent["config"]: # Check if qn is located on one of the tree vertices; if not, it is located in the tree swath or on one edge between "vertex" and "parent" nodes
                    # Split the edge on which qn is located
                    del g.edges[edge_index]
                    g.adjacency[vertex["id"]].remove(parent["id"])
                    g.adjacency[parent["id"]].remove(vertex["id"])
                    id = g.number_vertices
                    # Add qn as a vertex to the tree
                    g.add_vertex({"id": id, "config": qn})
                    # Connect qn to "parent" node
                    g.add_edge({"id": id, "config": qn}, parent)
                    # Connect "vertex" node to qn
                    g.add_edge(vertex, {"id": id, "config": qn})
                    # Now, add qs and connect it to qn
                    id_new = g.number_vertices
                    g.add_vertex({"id": id_new, "config": qs})
                    g.add_edge({"id": id_new, "config": qs}, {"id": id, "config": qn})
                elif qn == vertex["config"]: # Check if qn is located on "vertex"; if yes, do not add qn as a new vertex, just connect qs to qn (which is "vertex" node)
                    id_new = g.number_vertices
                    g.add_vertex({"id": id_new, "config": qs})
                    g.add_edge({"id": id_new, "config": qs}, vertex)
                elif qn == parent["config"]: # Check if qn is located on "parent"; if yes, do not add qn as a new vertex, just connect qs to qn (which is "parent" node)
                    id_new = g.number_vertices
                    g.add_vertex({"id": id_new, "config": qs})
                    g.add_edge({"id": id_new, "config": qs}, parent)
            if qs == qG: # Check if qs is equal to the goal configuration qG; if yes, find the path from qI to qG
                # The path finding algorithm is easy since RRT makes sure that the graph is Tree (each vertex has only one parent vertex) and the parent is stored as the first vertex id in the graph adjacency list
                path.append(id_new) # Begin from the last added vertex (with "id" = "id_new" and "config" = qs = qG)
                current = id_new
                if qI == qG:
                    return (g, path)
                while current != 0:
                    id2 = g.adjacency[current][0]
                    path.append(id2)
                    current = id2
                path.reverse() # To reverse the found path (which is the list of vertices ids from qG to qI, but the question asked for qI to qG)
                return (g, path)
    else: # If there is no specific query or tuple of (qI, qG) as the target of the algorithm (just expanding a tree in the C_free space of the problem until the maximum number of iterations is reached)
        # This block of the code is almost the same as the above "if" block except that it does not have the parts related to qG and path finding
        for i in range(max_samples):
            alpha_i = s.sample_free()
            min_distance = math.inf
            qn = copy.deepcopy(qI)
            vertex = {"id": g.number_vertices, "config": qI}
            parent = {"id": g.number_vertices, "config": qI}
            edge_index = 0
            for counter, edge in enumerate(g.edges):
                (q, distance) = geometry.get_nearest_point_on_line(g.vertices[edge[0]]["config"], g.vertices[edge[1]]["config"], alpha_i, resolution)
                if distance < min_distance:
                    min_distance = distance
                    qn = copy.deepcopy(q)
                    vertex = {"id": edge[0], "config": g.vertices[edge[0]]["config"]}
                    parent = {"id": edge[1], "config": g.vertices[edge[1]]["config"]}
                    edge_index = counter
            qs = stopping_config(s, qn, alpha_i, resolution)
            if qs != qn:
                if qn != vertex["config"] and qn != parent["config"]:
                    del g.edges[edge_index]
                    g.adjacency[vertex["id"]].remove(parent["id"])
                    g.adjacency[parent["id"]].remove(vertex["id"])
                    id = g.number_vertices
                    g.add_vertex({"id": id, "config": qn})
                    g.add_edge({"id": id, "config": qn}, parent)
                    g.add_edge(vertex, {"id": id, "config": qn})
                    id_new = g.number_vertices
                    g.add_vertex({"id": id_new, "config": qn})
                    g.add_edge({"id": id_new, "config": qn}, {"id": id, "config": qn})
                elif qn == vertex["config"]:
                    id = g.number_vertices
                    g.add_vertex({"id": id, "config": qs})
                    g.add_edge({"id": id, "config": qs}, vertex)
                elif qn == parent["config"]:
                    id = g.number_vertices
                    g.add_vertex({"id": id, "config": qs})
                    g.add_edge({"id": id, "config": qs}, parent)
    return (g, path)


def connect(s, q1_line, q2_line, delta_q): # Not used in the Midterm!
    """Auxiliary function to PRM to check if q1_line can be connected to q2_line without any collsions with
    the obstacles as encoded inside the "s" object"""
    direction = []
    length = geometry.get_euclidean_distance(q1_line, q2_line)
    for i in range(len(q1_line)):
        direction.append((q2_line[i]-q1_line[i])/length)
    i = 0
    qs_new = list(copy.deepcopy(q1_line))
    while length > 0:
        if length < delta_q and i > 0:
            for j in range(len(q2_line)):
                qs_new[j] = q2_line[j]
            if s.empty_collision_checker(qs_new) and s.obstacle_free(qs_new):
                return True
            else:
                return False
        else:
            for j in range(len(q1_line)):
                qs_new[j] = q1_line[j] + i * delta_q * direction[j]
            if not (s.empty_collision_checker(qs_new) and s.obstacle_free(qs_new)):
                return False
            length = length - delta_q
            i = i + 1
    return True


def PRM(world, obstacles, query, max_samples = 1000, resolution = 0.1, k = 15): # Not used in the Midterm!
    """Main PRM function which generates an exploring graph and possibly a path connecting qI to qG as
    defined by query (if query is empty, it just generates the graph)
    
    @returns: a tuple of the exploring graph "g" and the found "path" ("path" is empty if there is no
              query or the path could not be found given the maximum sampling iterations).
    """
    s = search_space(world, obstacles)
    g = graph.Graph()

    path = []
    alpha_i = s.sample_free()
    g.add_vertex(alpha_i)
    for i in range(1, max_samples):
        alpha_i = s.sample_free()
        g.add_vertex(alpha_i)
        distances = [0]*len(g.vertices)
        for i in range(len(g.vertices)):
            distances[i] = geometry.get_euclidean_distance(g.vertices[i], alpha_i)
        vertices = [vertex for _, vertex in sorted(zip(distances, g.vertices), key=lambda pair: pair[0])]
        limit = k + 1 if len(vertices) >= k + 1 else len(vertices)
        for i in range(1, limit):
            if not(g.same_component(alpha_i, vertices[i], k)) and connect(s, vertices[i], alpha_i, resolution):
                g.add_edge(alpha_i, vertices[i])
    if query is not None:
        qI = tuple(query[0])
        qG = tuple(query[1])

        alpha_i = qI
        if alpha_i not in g.vertices:
            g.add_vertex(alpha_i)
            distances = [0]*len(g.vertices)
            for i in range(len(g.vertices)):
                distances[i] = geometry.get_euclidean_distance(g.vertices[i], alpha_i)
            vertices = [vertex for _, vertex in sorted(zip(distances, g.vertices), key=lambda pair: pair[0])]
            limit = k + 1 if len(vertices) >= k + 1 else len(vertices)
            for i in range(1, limit):
                if not(g.same_component(alpha_i, vertices[i], k)) and connect(s, vertices[i], alpha_i, resolution):
                    g.add_edge(alpha_i, vertices[i])
        
        alpha_i = qG
        if alpha_i not in g.vertices:
            g.add_vertex(alpha_i)
            distances = [0]*len(g.vertices)
            for i in range(len(g.vertices)):
                distances[i] = geometry.get_euclidean_distance(g.vertices[i], alpha_i)
            vertices = [vertex for _, vertex in sorted(zip(distances, g.vertices), key=lambda pair: pair[0])]
            limit = k + 1 if len(vertices) >= k + 1 else len(vertices)
            for i in range(1, limit):
                if not(g.same_component(alpha_i, vertices[i], k)) and connect(s, vertices[i], alpha_i, resolution):
                    g.add_edge(alpha_i, vertices[i])
        
        X = graph.Grid2DStates(g, world.get_bounds()[0][0], world.get_bounds()[0][1], world.get_bounds()[1][0], world.get_bounds()[1][1], [])
        f = graph.GridStateTransition(g)
        U = graph.Grid2DActions(X, f)
        search_result = fsearch(X, U, f, qI, [qG], ALG_BFS)
        path = search_result["path"]
    
    return (g, path)


if __name__ == "__main__":
    # Preprocessing: extracting the input and hyperparemeters used to run this project
    args = parse_args()
    (O, dt, ORIGIN, W, H, L, D, QUERIES, MAX_SAMPLES, RESOLUTION, PROBABILITY_CONNECT_GOAL, K) = parse_desc(args.desc)
    if L == 0:
        world = geometry.world_2D_obstacle(ORIGIN, W, H)
    else:
        world = geometry.world_2D_obstacle(ORIGIN, H, H)
    obstacles = []
    obstacles_boundary = []
    for i in range(len(O)):
        if len(O[i]) == 2:
            c = geometry.circular_obstacle(O[i][0], O[i][1]-dt)
        else:
            c = geometry.rectangular_obstacle(O[i], W, L, D)
        obstacles.append(c)
        obstacles_boundary.append(c.get_boundary())
    g = None
    path = []

    # # Part 1: RRT exploaration, neglecting obstacles (HW4)
    # fig, ax = plt.subplots()
    # [g, path] = RRT(world, None, None, MAX_SAMPLES, RESOLUTION, PROBABILITY_CONNECT_GOAL)
    # X = graph.Grid2DStates(g, world.get_bounds()[0][0], world.get_bounds()[0][1], world.get_bounds()[1][0], world.get_bounds()[1][1], [])
    # draw(ax, world.get_bounds(), obstacles_boundary, tuple(QUERIES[0][0]), tuple(QUERIES[0][1]), X, [g.vertices[id]["config"] for id in path], title="RRT exploaration, neglecting obstacles")
    # plt.show()

    # # Part 2: RRT exploaration, considering obstacles (HW4)
    # fig, ax = plt.subplots()
    # [g, path] = RRT(world, obstacles, None, MAX_SAMPLES, RESOLUTION, PROBABILITY_CONNECT_GOAL)
    # X = graph.Grid2DStates(g, world.get_bounds()[0][0], world.get_bounds()[0][1], world.get_bounds()[1][0], world.get_bounds()[1][1], [])
    # draw(ax, world.get_bounds(), obstacles_boundary, tuple(QUERIES[0][0]), tuple(QUERIES[0][1]), X, [g.vertices[id]["config"] for id in path], title="RRT exploaration, considering obstacles")
    # plt.show()

    # RRT planning (modified from HW4)
    for i in range(len(QUERIES)):
        fig, ax = plt.subplots()
        [g, path] = RRT(world, obstacles, QUERIES[i], MAX_SAMPLES, RESOLUTION, PROBABILITY_CONNECT_GOAL)
        X = graph.Grid2DStates(g, world.get_bounds()[0][0], world.get_bounds()[0][1], world.get_bounds()[1][0], world.get_bounds()[1][1], [])
        draw(ax, world.get_bounds(), obstacles_boundary, tuple(QUERIES[i][0]), tuple(QUERIES[i][1]), X, [g.vertices[id]["config"] for id in path], title="RRT planning for a 2-link robot")
        print(path)
        plt.show()

    # # Part 4: PRM planning (HW4)
    # for i in range(len(QUERIES)):
    #     fig, ax = plt.subplots()
    #     [g, path] = PRM(world, obstacles, QUERIES[i], MAX_SAMPLES, RESOLUTION, K)
    #     X = graph.Grid2DStates(g, world.get_bounds()[0][0], world.get_bounds()[0][1], world.get_bounds()[1][0], world.get_bounds()[1][1], [])
    #     draw(ax, world.get_bounds(), obstacles_boundary, tuple(QUERIES[i][0]), tuple(QUERIES[i][1]), X, [g.vertices[id]["config"] for id in path], title="PRM planning")
    #     print(path)
    #     plt.show()
    
    # Outputing the results as requested in the Midterm doc
    if g is None:
        result = {"vertices": None, "edges": None, "path": None}
    else:
        result = {"vertices": g.vertices, "edges": g.edges, "path": path}

    with open(args.out, "w") as outfile:
        json.dump(result, outfile)

