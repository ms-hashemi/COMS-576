import sys, math, argparse
import numpy as np
import matplotlib.pyplot as plt
from planning import (
    rrt,
    rrt_optimal,
    prm,
    prm_optimal,
    EdgeCreator,
    DistanceComputator,
    ObstacleCollisionChecker,
)
from dubins import shortest_path
from edge import Edge
from obstacle import construct_circular_obstacles, WorldBoundary2D
from draw_cspace import draw

ALG_RRT = "rrt"
ALG_PRM = "prm"
ALG_RRT_OPTIMAL = "rrt_optimal"
ALG_PRM_OPTIMAL = "prm_optimal"


class DubinsEdgeCreator(EdgeCreator):
    def __init__(self, rho_min, step_size):
        self.rho_min = rho_min
        self.step_size = step_size

    def make_edge(self, s1, s2):
        return EdgeDubins(s1, s2, self.rho_min, self.step_size)


class DubinsDistanceComputator(DistanceComputator):
    def __init__(self, rho_min):
        self.rho_min = rho_min

    def get_distance(self, s1, s2):
        """Return the Euclidean distance between s1 and s2"""
        path = shortest_path(s1, s2, self.rho_min)
        return path.path_length()


class EdgeDubins(Edge):
    """Store the information about an edge representing a Dubins curve between 2 points"""

    def __init__(self, s1, s2, rho_min, step_size=0.5, length=None, states=None):
        super().__init__(s1, s2, step_size)

        # The shortest dubins curve and the discretized points along the path
        self.rho_min = rho_min
        if length is None or states is None:
            self._update_path()
        else:
            self.length = length
            self.states = states

    def _update_path(self):
        path = shortest_path(self.s1, self.s2, self.rho_min)
        self.length = path.path_length()

        # Change the step_size to make equal spacing between consecutive states
        # First, compute the number of states (excluding the first one) so that the new step_size
        # is not larger than teh given step_size
        num_states = math.ceil(self.length / self.step_size)
        self.step_size = self.length / num_states

        (self.states, _) = path.sample_many(self.step_size)
        self.states = [np.array(state) for state in self.states]

        # Make sure the last state s2 is also in self.states (possibly missing due to
        # numerical error
        if len(self.states) < num_states + 1:
            self.states.append(self.s2)
        assert len(self.states) == num_states + 1

    def get_path(self):
        """Return the path, representing the geometry of the edge"""
        return self.states

    def reverse(self):
        """Reverse the origin/destination of the edge"""
        super().reverse()
        self._update_path()

    def get_discretized_state(self, i):
        """Return the i^{th} discretized state"""
        if i >= len(self.states):
            return None
        return self.states[i]

    def get_nearest_point(self, state):
        """Compute the nearest point on this edge to the given state

        @return (s, t) where s is the point on the edge that is closest to state
        and it is at distance t*length from the beginning of the edge
        """
        nearest_dist = math.inf
        nearest_ind = None
        for ind, s in enumerate(self.states):
            path = shortest_path(s, state, self.rho_min)
            dist = path.path_length()
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_ind = ind

        if nearest_ind == 0:
            return (self.s1, 0)

        if nearest_ind == len(self.states) - 1:
            return (self.s2, 1)

        t = nearest_ind * self.step_size / self.length
        assert t > 0 and t < 1
        return (self.states[nearest_ind], t)

    def split(self, t):
        """Split the edge at distance t/length where length is the length of this edge

        @return (edge1, edge2) edge1 and edge2 are the result of splitting the original edge
        """
        split_length = t * self.length
        split_ind = round(split_length / self.step_size)
        assert split_ind > 0 and split_ind < len(self.states) - 1

        s = self.states[split_ind]
        edge1 = EdgeDubins(
            self.s1,
            s,
            self.rho_min,
            self.step_size,
            length=split_length,
            states=self.states[0 : split_ind + 1],
        )
        edge2 = EdgeDubins(
            s,
            self.s2,
            self.rho_min,
            self.step_size,
            length=self.length - split_length,
            states=self.states[split_ind:],
        )

        return (edge1, edge2)

    def get_length(self):
        """Return the length of the edge"""
        return self.length


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


if __name__ == "__main__":
    rho_min = 0.5
    cspace = [(-3, 3), (-1, 1), (-math.pi / 2, math.pi / 2)]
    qI = (-2, -0.5, 0)
    qG = (2, -0.5, math.pi / 2)
    obstacles = construct_circular_obstacles(0.2)
    obs_boundaries = [obstacle.get_boundaries() for obstacle in obstacles]
    world_boundary = WorldBoundary2D(cspace[0], cspace[1])
    obstacles.append(world_boundary)
    edge_creator = DubinsEdgeCreator(rho_min, 0.1)
    collision_checker = ObstacleCollisionChecker(obstacles)
    distance_computator = DubinsDistanceComputator(rho_min)

    args = parse_args()
    if args.alg == ALG_RRT:
        title = "RRT planning"
        (G, root, goal) = rrt(
            cspace=cspace,
            qI=qI,
            qG=qG,
            edge_creator=edge_creator,
            distance_computator=distance_computator,
            collision_checker=collision_checker,
        )
    elif args.alg == ALG_RRT_OPTIMAL:
        title = "RRT_OPTIMAL planning"
        (G, root, goal) = rrt_optimal(
            cspace=cspace,
            qI=qI,
            qG=qG,
            edge_creator=edge_creator,
            distance_computator=distance_computator,
            collision_checker=collision_checker,
        )
    elif args.alg == ALG_PRM:
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
    elif args.alg == ALG_PRM_OPTIMAL:
        title = "PRM_OPTIMAL planning"
        (G, root, goal) = prm_optimal(
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

    fig, ax = plt.subplots(1, 1)
    draw(ax, cspace, obs_boundaries, qI, qG, G, path, title)
    plt.show()
