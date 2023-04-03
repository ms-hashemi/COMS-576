#############################################################################################################
# COM S 476/576 Project 4 Solution
# Mohammad Hashemi
#############################################################################################################

import json, sys, os, argparse
import math
import numpy as np
import copy
import matplotlib.pyplot as plt
from discrete_search import fsearch, ALG_BFS
from draw_cspace import draw
import geometry
import graph


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

    O = data["O"]
    dt = data["dt"]
    ORIGIN = data["ORIGIN"]
    W = data["W"]
    H = data["H"]
    QUERIES = data["QUERIES"]
    MAX_SAMPLES = data["MAX_SAMPLES"]
    RESOLUTION = data["RESOLUTION"]
    PROBABILITY_CONNECT_GOAL = data["PROBABILITY_CONNECT_GOAL"]
    K = data["K"]
    return (O, dt, ORIGIN, W, H, QUERIES, MAX_SAMPLES, RESOLUTION, PROBABILITY_CONNECT_GOAL, K)


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
    """Auxiliary function to RRT to find the point on the new edge before obstacle"""
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
    g.add_vertex(qI)

    path = []
    if query is not None:
        for i in range(max_samples):
            if i > 0 and i % 50 == 0:
                alpha_i = qG
            else:
                if np.random.uniform(0, 1) > p:
                    alpha_i = s.sample_free()
                else:
                    alpha_i = qG
            min_distance = math.inf
            qn = copy.deepcopy(qI)
            vertex = copy.deepcopy(qI)
            parent = copy.deepcopy(qI)
            for key, value in g.edges.items():
                (q, distance) = geometry.get_nearest_point_on_line(key, value[0], alpha_i, resolution)
                if distance < min_distance:
                    min_distance = distance
                    qn = copy.deepcopy(q)
                    vertex = copy.deepcopy(key)
                    parent = copy.deepcopy(value[0])
            qs = stopping_config(s, qn, alpha_i, resolution)
            if qs != qn:
                if qn != vertex and qn != parent:
                    g.add_vertex(qn)
                    g.add_edge(qn, parent)
                    g.edges[vertex][0] = qn
                    g.add_vertex(qs)
                    g.add_edge(qs, qn)
                else:
                    g.add_vertex(qs)
                    g.add_edge(qs, qn)
            if qs == qG:
                path.append(qG)
                current = qG
                if qI == qG:
                    return (g, path)
                while not g.edges[current][0] == qI:
                    path.append(g.edges[current][0])
                    current = g.edges[current][0]
                path.append(qI)
                path.reverse()
                return (g, path)
    else:
        for i in range(max_samples):
            # print(i)
            alpha_i = s.sample_free()
            min_distance = math.inf
            qn = copy.deepcopy(qI)
            vertex = copy.deepcopy(qI)
            parent = copy.deepcopy(qI)
            for key, value in g.edges.items():
                (q, distance) = geometry.get_nearest_point_on_line(key, value[0], alpha_i, resolution)
                if distance < min_distance:
                    min_distance = distance
                    qn = copy.deepcopy(q)
                    vertex = copy.deepcopy(key)
                    parent = copy.deepcopy(value[0])
            qs = stopping_config(s, qn, alpha_i, resolution)
            if qs != qn:
                if qn != vertex and qn != parent:
                    g.add_vertex(qn)
                    g.add_edge(qn, parent)
                    g.edges[vertex][0] = qn
                    g.add_vertex(qs)
                    g.add_edge(qs, qn)
                else:
                    g.add_vertex(qs)
                    g.add_edge(qs, qn)
    return (g, path)


def connect(s, q1_line, q2_line, delta_q):
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


def PRM(world, obstacles, query, max_samples = 1000, resolution = 0.1, k = 15):
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
        # print(i)
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
    (O, dt, ORIGIN, W, H, QUERIES, MAX_SAMPLES, RESOLUTION, PROBABILITY_CONNECT_GOAL, K) = parse_desc(args.desc)

    world = geometry.world_2D_obstacle(ORIGIN, W, H)
    obstacles = []
    obstacles_boundary = []
    for i in range(len(O)):
        c = geometry.circular_obstacle(O[i][0], O[i][1]-dt)
        obstacles.append(c)
        obstacles_boundary.append(c.get_boundary())

    # Part 1: RRT exploaration, neglecting obstacles
    fig, ax = plt.subplots()
    [g, path] = RRT(world, None, None, MAX_SAMPLES, RESOLUTION, PROBABILITY_CONNECT_GOAL)
    X = graph.Grid2DStates(g, world.get_bounds()[0][0], world.get_bounds()[0][1], world.get_bounds()[1][0], world.get_bounds()[1][1], [])
    draw(ax, world.get_bounds(), obstacles_boundary, tuple(QUERIES[0][0]), tuple(QUERIES[0][1]), X, path, title="RRT exploaration, neglecting obstacles")
    plt.show()

    # Part 2: RRT exploaration, considering obstacles
    fig, ax = plt.subplots()
    [g, path] = RRT(world, obstacles, None, MAX_SAMPLES, RESOLUTION, PROBABILITY_CONNECT_GOAL)
    X = graph.Grid2DStates(g, world.get_bounds()[0][0], world.get_bounds()[0][1], world.get_bounds()[1][0], world.get_bounds()[1][1], [])
    draw(ax, world.get_bounds(), obstacles_boundary, tuple(QUERIES[0][0]), tuple(QUERIES[0][1]), X, path, title="RRT exploaration, considering obstacles")
    plt.show()

    # Part 3: RRT planning
    for i in range(len(QUERIES)):
        fig, ax = plt.subplots()
        [g, path] = RRT(world, obstacles, QUERIES[i], MAX_SAMPLES, RESOLUTION, PROBABILITY_CONNECT_GOAL)
        X = graph.Grid2DStates(g, world.get_bounds()[0][0], world.get_bounds()[0][1], world.get_bounds()[1][0], world.get_bounds()[1][1], [])
        draw(ax, world.get_bounds(), obstacles_boundary, tuple(QUERIES[i][0]), tuple(QUERIES[i][1]), X, path, title="RRT planning")
        print(path)
        plt.show()

    # Part 4: PRM planning
    for i in range(len(QUERIES)):
        fig, ax = plt.subplots()
        [g, path] = PRM(world, obstacles, QUERIES[i], MAX_SAMPLES, RESOLUTION, K)
        X = graph.Grid2DStates(g, world.get_bounds()[0][0], world.get_bounds()[0][1], world.get_bounds()[1][0], world.get_bounds()[1][1], [])
        draw(ax, world.get_bounds(), obstacles_boundary, tuple(QUERIES[i][0]), tuple(QUERIES[i][1]), X, path, title="PRM planning")
        print(path)
        plt.show()
    
    # result = {"path": search_result["path"]}

    # with open(args.out, "w") as outfile:
    #     json.dump(result, outfile)

