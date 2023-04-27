#!/usr/bin/env python
import math
import rospy
import numpy as np
import matplotlib.pyplot as plt
from cs476.msg import Chain2D


def get_chain_msg():
    """Return a message from the "chain_config" channel.

    This function will wait until a message is received.
    """
    rospy.init_node("chain_plotter", anonymous=True)
    chain = rospy.wait_for_message("chain_config", Chain2D)
    return chain


def plot_chain(config, W, L, D):
    """Plot a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link
    """

    (joint_positions, link_vertices) = get_link_positions(config, W, L, D)

    fig, ax = plt.subplots()
    plot_links(link_vertices, ax)
    plot_joints(joint_positions, ax)
    ax.axis("equal")
    plt.show()


def plot_links(link_vertices, ax):
    """Plot the links of a 2D kinematic chain A_1, ..., A_m on the axis ax

    @type link_vertices: a list [V_1, ..., V_m] where V_i is the list of [x,y] positions of vertices of A_i
    """

    for vertices in link_vertices:
        x = [vertex[0] for vertex in vertices]
        y = [vertex[1] for vertex in vertices]

        x.append(vertices[0][0])
        y.append(vertices[0][1])
        ax.plot(x, y, "k-", linewidth=2)


def plot_joints(joint_positions, ax):
    """Plot the joints of a 2D kinematic chain A_1, ..., A_m on the axis ax

    @type joint_positions: a list [p_1, ..., p_{m+1}] where p_i is the position [x,y] of the joint between A_i and A_{i-1}
    """
    x = [pos[0] for pos in joint_positions]
    y = [pos[1] for pos in joint_positions]
    ax.plot(x, y, "k.", markersize=10)


def get_link_positions(config, W, L, D):
    """Compute the positions of the links and the joints of a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link

    @return: a tuple (joint_positions, link_vertices) where
        * joint_positions is a list [p_1, ..., p_{m+1}] where p_i is the position [x,y] of the joint between A_i and A_{i-1}
        * link_vertices is a list [V_1, ..., V_m] where V_i is the list of [x,y] positions of vertices of A_i
    """

    if len(config) == 0:
        return ([], [])

    joint_positions = [np.array([0, 0, 1])]
    link_vertices = []

    link_vertices_body = [
        np.array([-(L - D) / 2, -W / 2, 1]),
        np.array([D + (L - D) / 2, -W / 2, 1]),
        np.array([D + (L - D) / 2, W / 2, 1]),
        np.array([-(L - D) / 2, W / 2, 1]),
    ]
    joint_body = np.array([D, 0, 1])
    trans_mat = np.array(
        [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
        ]
    )

    for i in range(len(config)):
        a = D if i > 0 else 0
        trans_mat = np.matmul(trans_mat, get_trans_mat(config[i], a))
        joint = np.matmul(trans_mat, joint_body)
        vertices = [
            np.matmul(trans_mat, link_vertex) for link_vertex in link_vertices_body
        ]
        joint_positions.append(joint)
        link_vertices.append(vertices)

    return (joint_positions, link_vertices)


def get_trans_mat(theta, a):
    """Return the homogeneous transformation matrix"""
    return np.array(
        [
            [math.cos(theta), -math.sin(theta), a],
            [math.sin(theta), math.cos(theta), 0],
            [0, 0, 1],
        ]
    )


if __name__ == "__main__":
    chain = get_chain_msg()
    plot_chain(chain.config, chain.W, chain.L, chain.D)
