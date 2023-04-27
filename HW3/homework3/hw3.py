#############################################################################################################
# COM S 476/576 Homework 3 Solution
# Tichakorn Wongpiromsarn
#############################################################################################################

import json, sys, os, argparse, math
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from hw2_chain_plotter import get_link_positions
from discrete_search import fsearch, ALG_BFS
from hw1 import Grid2DStates, GridStateTransition, Grid2DActions, draw_path


LINK_ANGLES = range(-180, 180)


def is_in_collision(config, W, L, D, obs_polygons):
    """Determine whether the robot at the given configuration is in collision with obstacle
    @type config: a tuple that represents the configuration of the robot where each angle is
                  in degree
    @type W:   float, representing the width of each link
    @type L:   float, representing the length of each link
    @type D:   float, the distance between the two points of attachment on each link
    @type obs_polygons: a list of shapely Polygon objects representing the obstacles

    """
    # Convert the configuration from degree to radian
    config = tuple(config[i] * (math.pi / 180.0) for i in range(len(config)))

    (_, all_link_vertices) = get_link_positions(config, W, L, D)
    for link_vertices in all_link_vertices:
        link = Polygon(link_vertices)
        for obs in obs_polygons:
            if link.intersects(obs):
                return True
    return False


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
    Cobs = []
    obs_polygons = [Polygon(obs_vertices) for obs_vertices in O]

    for theta1 in LINK_ANGLES:
        for theta2 in LINK_ANGLES:
            config = (theta1, theta2)
            if is_in_collision(config, W, L, D, obs_polygons):
                Cobs.append((theta1, theta2))

    return Cobs


def compute_Cfree(Cobs):
    """Compute the free space for a 2-link robot

    @type Cobs: a list of configurations (theta_1, theta_2) of the robot that leads to a collision
                between the robot and an obstacle in O.

    @return an instance of Grid2DStates that represents the free space
    """
    return Grid2DStates(
        LINK_ANGLES[0], LINK_ANGLES[-1], LINK_ANGLES[0], LINK_ANGLES[-1], Cobs
    )


def get_discretized_path(config1, config2, resolution):
    """Return the discretization of a line segment between config1 and config2 with the given resolution"""
    dconfig = tuple(config2[i] - config1[i] for i in range(len(config1)))
    dmax = max([abs(dconfig[i]) for i in range(len(dconfig))])
    num_configs = math.ceil(dmax / resolution)
    # Discretization of the path between config1 and config2,
    # including config1 but not including config2
    return [
        tuple(config1[i] + dconfig[i] * j / num_configs for i in range(len(config1)))
        for j in range(num_configs)
    ]


def check_collision(path, O, W, L, D, resolution=0.1):
    """Return a list of configurations of the robot along the path that cause the robot to be in collision

    @type path: a list of configurations (theta_1, theta_2) where theta_i is in degree
    @type O:   a list of obstacles, where for each i, O[i] is a list [(x_0, y_0), ..., (x_m, y_m)]
               of coordinates of the vertices of the i^th obstacle
    @type W:   float, representing the width of each link
    @type L:   float, representing the length of each link
    @type D:   float, the distance between the two points of attachment on each link
    """
    path_length = len(path)
    if path_length == 0:
        return []

    obs_polygons = [Polygon(obs_vertices) for obs_vertices in O]
    colliding_configs = []

    for path_ind in range(path_length - 1):
        discretized_path = get_discretized_path(
            path[path_ind], path[path_ind + 1], resolution
        )
        for config in discretized_path:
            if is_in_collision(config, W, L, D, obs_polygons):
                colliding_configs.append(config)

    # Check the last configuration on the path
    if is_in_collision(path[-1], W, L, D, obs_polygons):
        colliding_configs.append(path[-1])

    return colliding_configs


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
    W = data["W"]
    L = data["L"]
    D = data["D"]
    xI = tuple(data["xI"])
    XG = [tuple(x) for x in data["XG"]]
    U = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    return (O, W, L, D, xI, XG, U)


if __name__ == "__main__":
    args = parse_args()
    (O, W, L, D, xI, XG, U) = parse_desc(args.desc)
    Cobs = compute_Cobs(O, W, L, D)

    X = compute_Cfree(Cobs)
    f = GridStateTransition()
    U = Grid2DActions(X, f)

    search_result = fsearch(X, U, f, xI, XG, ALG_BFS)
    colliding_configs = check_collision(search_result["path"], O, W, L, D, 0.1)
    print(
        "Configurations along the path where the robot is in collision: ",
        colliding_configs,
    )

    result = {"Cobs": Cobs, "path": search_result["path"]}

    with open(args.out, "w") as outfile:
        json.dump(result, outfile)

    fig, ax = plt.subplots()
    X.draw(ax, grid_on=False, tick_step=[30, 30])
    draw_path(ax, search_result["path"])
    plt.show()
