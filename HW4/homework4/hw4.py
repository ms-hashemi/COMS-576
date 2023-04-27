import sys, argparse
import matplotlib.pyplot as plt
from planning import (
    rrt,
    prm,
    StraightEdgeCreator,
    EuclideanDistanceComputator,
    EmptyCollisionChecker,
    ObstacleCollisionChecker,
)
from obstacle import construct_circular_obstacles, WorldBoundary2D
from draw_cspace import draw

ALG_RRT = "rrt"
ALG_PRM = "prm"


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Run sampling-based motion planning algorithm"
    )
    parser.add_argument(
        "--alg",
        choices=[ALG_RRT, ALG_PRM],
        required=False,
        default=ALG_RRT,
        dest="alg",
        help="algorithm, default to rrt",
    )
    args = parser.parse_args(sys.argv[1:])
    return args


def main_rrt(
    cspace, qI, qG, edge_creator, distance_computator, collision_checker, obs_boundaries
):
    """Task 1 (Exploring the C-space using RRT) and Task 2 (Solve the planning problem using RRT)"""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

    # Task 1a: Neglect obstacles and goal
    title1 = "RRT exploration, neglecting obstacles"
    (G1, _, _) = rrt(
        cspace=cspace,
        qI=qI,
        qG=None,
        edge_creator=edge_creator,
        distance_computator=distance_computator,
        collision_checker=EmptyCollisionChecker(),
    )
    draw(ax1, cspace, obs_boundaries, qI, qG, G1, [], title1)

    # Task 1b: Include obstacles, neglect goal
    title2 = "RRT exploration, considering obstacles"
    (G2, _, _) = rrt(
        cspace=cspace,
        qI=qI,
        qG=None,
        edge_creator=edge_creator,
        distance_computator=distance_computator,
        collision_checker=collision_checker,
    )
    draw(ax2, cspace, obs_boundaries, qI, qG, G2, [], title2)

    # Task 2: Include obstacles and goal
    title3 = "RRT planning"
    (G3, root3, goal3) = rrt(
        cspace=cspace,
        qI=qI,
        qG=qG,
        edge_creator=edge_creator,
        distance_computator=distance_computator,
        collision_checker=collision_checker,
    )
    path = []
    if goal3 is not None:
        path = G3.get_path(root3, goal3)
    draw(ax3, cspace, obs_boundaries, qI, qG, G3, path, title3)

    plt.show()


def main_prm(
    cspace, qI, qG, edge_creator, distance_computator, collision_checker, obs_boundaries
):
    """Task 3 (Solve the planning problem using PRM)"""
    fig, ax = plt.subplots()
    title = "PRM planning"
    (G, root, goal) = prm(
        cspace=cspace,
        qI=qI,
        qG=qG,
        edge_creator=edge_creator,
        distance_computator=distance_computator,
        collision_checker=collision_checker,
        k=15,
    )
    path = []
    if root is not None and goal is not None:
        path = G.get_path(root, goal)
    draw(ax, cspace, obs_boundaries, qI, qG, G, path, title)
    plt.show()


if __name__ == "__main__":
    cspace = [(-3, 3), (-1, 1)]
    qI = (-2, -0.5)
    qG = (2, -0.5)
    obstacles = construct_circular_obstacles(0.02)
    obs_boundaries = [obstacle.get_boundaries() for obstacle in obstacles]

    # We don't really need to explicitly need to check the world boundary
    # because the world is convex and we're connecting points by a straight line.
    # So as long as the two points are within the world, the line between them
    # are also within the world.
    # I'm just including this for completeness.
    world_boundary = WorldBoundary2D(cspace[0], cspace[1])
    obstacles.append(world_boundary)

    edge_creator = StraightEdgeCreator(0.1)
    collision_checker = ObstacleCollisionChecker(obstacles)
    distance_computator = EuclideanDistanceComputator()

    args = parse_args()

    if args.alg == ALG_RRT:
        main_rrt(
            cspace,
            qI,
            qG,
            edge_creator,
            distance_computator,
            collision_checker,
            obs_boundaries,
        )
    else:
        main_prm(
            cspace,
            qI,
            qG,
            edge_creator,
            distance_computator,
            collision_checker,
            obs_boundaries,
        )
