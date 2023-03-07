#############################################################################################################
# COM S 476/576 Project 2 Solution
# Tichakorn Wongpiromsarn
#############################################################################################################

import json, sys, os, argparse
import matplotlib.pyplot as plt
from discrete_search import fsearch, ALG_BFS
from hw1 import Grid2DStates, GridStateTransition, Grid2DActions, draw_path


LINK_ANGLES = [i - 180 for i in range(360)]


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
    import hw2_chain_plotter # Importing this code from HW2 for the transformations of the robot

    Cobs = [] # The only output of this function which is the space of obstacle configurations
    obstacles = [shapely.Polygon(O[i]) for i in range(len(O))] # Obstacles are all polygons

    # Iterating over all possible configurations in the problem world
    for theta_1 in range(-180, 180, 1):
        for theta_2 in range(-180, 180, 1):
            config = [theta_1*numpy.pi/180, theta_2*numpy.pi/180]
            (joint_positions, link_vertices) = hw2_chain_plotter.get_link_positions(config, W, L, D) # Transforming the robot (all linkages) according to the current config
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


def compute_Cfree(Cobs):
    """Compute the free space for a 2-link robot

    @type Cobs: a list of configurations (theta_1, theta_2) of the robot that leads to a collision
                between the robot and an obstacle in O.

    @return an instance of Grid2DStates that represents the free space
    """
    # TODO: Implement this function
    # C_free = C_world - C_obstacles
    # Using HW1 class to define C_free after finding Cobs (through compute_Cobs(O, W, L, D))
    return Grid2DStates(-180, 180, -180, 180, Cobs)
    # raise NotImplementedError


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

    result = {"Cobs": Cobs, "path": search_result["path"]}

    with open(args.out, "w") as outfile:
        json.dump(result, outfile)

    fig, ax = plt.subplots()
    X.draw(ax, grid_on=False, tick_step=[30, 30])
    draw_path(ax, search_result["path"])
    plt.show()

    # This piece of code was written by me (not in the class template) for Task3 of HW3
    # It basically check find possible collision configurating on the re-discretized path to the solution
    # Discretization is linear with the limit of maximum increment of 0.1 (degrees) in any configuration components
    
    import numpy # Importing Numpy for math functions (including pi number)
    # Most probably, the following are not available conventionally, so plaese install "Shapely" and copy my "hw2_chain_plotter.py" file to have a functioning code!
    import shapely # A Python library for set-theoretic analysis and manipulation of planar features
    import hw2_chain_plotter # Importing this code from HW2 for the transformations of the robot

    obstacles = [shapely.Polygon(O[i]) for i in range(len(O))] # Obstacles are all polygons
    collision_configs_on_fpath = [] # The main output of this code piece, which is going to be constructed iteratively through the path discretization
    increment = 0.1
    for i in range(0, len(search_result["path"]), 2): # The loop goes over even-indexed elements of the found path since in each iteration, the path between two configs is going to be discretized again according to the "increment" limit
        point_on_path = numpy.array(search_result["path"][i]) # Current point on the path
        if i + 1 >= len(search_result["path"]): # Check if there exist a next point on the path
            break
        point_on_path_next = numpy.array(search_result["path"][i + 1]) # Next point on the path
        direction = point_on_path_next-point_on_path
        # Normalize the vector
        direction = direction/((direction[0]**2+direction[1]**2)**0.5) 
        # Discretization between "point_on_path" and "point_on_path_next" along "direction" considering the above max increment limit
        if direction[0] == 0:
            tau = increment
            index_min = 1 # This config is going to control the number of discretization along "direction"
        elif direction[1] == 0:
            tau = increment
            index_min = 0
        else:
            tau = numpy.min(increment/direction)
            index_min = numpy.argmin(increment/direction)
        for i in range(1, int(numpy.floor(direction[index_min]/tau))+2):
            mid_point = point_on_path + i * tau * direction
            mid_to_next_vector = point_on_path_next - mid_point
            if numpy.matmul(mid_to_next_vector, direction) > 0: # Check if we are still between the Current point and the Next point by a dot product
                # What follows is adopted from Task1 or "compute_Cobs(O, W, L, D)" above
                config = [mid_point[0]*numpy.pi/180, mid_point[1]*numpy.pi/180]
                (joint_positions, link_vertices) = hw2_chain_plotter.get_link_positions(config, W, L, D)
                linkages = [shapely.Polygon(link_vertices[i]) for i in range(len(link_vertices))]
                found_intersection = False
                for linkage in linkages:
                    for obstacle in obstacles:
                        if linkage.intersects(obstacle):
                            collision_configs_on_fpath.append((config[0], config[1]))
                            found_intersection = True
                            break
                    if found_intersection:
                        break
            else: # If we are not between the Current point and the Next point, we should break this discretization loop and move to the next point on the path (i.e., next discretization)
                break
    print(collision_configs_on_fpath)
