#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt


def get_chain_msg():
    """Return a message from the "chain_config" channel.

    This function will wait until a message is received.
    """
    # TODO: Implement this function
    from cs476.msg import Chain2D
    
    rospy.init_node("listener", anonymous=True)
    msg = rospy.wait_for_message("chain_config", Chain2D)
    # msg_str = " ".join(map(str, msg.nums))
    # rospy.loginfo(rospy.get_caller_id() + " I heard %s", msg_str)
    rospy.loginfo(rospy.get_caller_id() + "\nConfig = %s\n     W = %s\n     L = %s\n     D = %s", msg.config, msg.W, msg.L, msg.D)
    return msg
    # raise NotImplementedError


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
    # TODO: Implement this function
    # Importing "numpy" library for math functions ("sin", "cos") and matrix multiplication ("matmul")
    import numpy

    joint_positions = [] # The positions of all joints in a list with repect to the world frame
    link_vertices = [] # The positions of all vertices of all linkages in a list (of lists) with repect to the world frame
    rot = numpy.zeros((1, 3, 3)) # Initializing the rotation matrix (or matrices) as a numpy matrix (the first one is just a placeholder to be appended by later rotation matrices)
    if len(config) > 0:
        joint_positions.append([0.0, 0.0]) # The absolute or world frame position of the first joint is always at the center of the world frame per convention/problem statement
    # Calculating all linkages' positions in the world frame
    joint_relative = [0.0, 0.0] # The relative or current frame position of the first or '0'th joint
    for i in range(len(config)):
        theta = config[i]
        rot = numpy.append(rot, numpy.array([[[numpy.cos(theta), -numpy.sin(theta), joint_relative[0]], [numpy.sin(theta), numpy.cos(theta), joint_relative[1]], [0.0, 0.0, 1.0]]]), axis = 0)
        joint_relative = [D, 0.0]
        joint = numpy.expand_dims(numpy.array([D, 0.0, 1.0]).T, axis = 1) # The relative or current frame position of the next or 'i+1'th joint
        v_1 = numpy.expand_dims(numpy.array([-(L-D)/2, -W/2, 1.0]).T, axis = 1) # The relative or current frame position of the first vertex of the current linkage
        v_2 = numpy.expand_dims(numpy.array([D+(L-D)/2, -W/2, 1.0]).T, axis = 1) # The relative or current frame position of the second vertex of the current linkage
        v_3 = numpy.expand_dims(numpy.array([D+(L-D)/2, +W/2, 1.0]).T, axis = 1) # The relative or current frame position of the third vertex of the current linkage
        v_4 = numpy.expand_dims(numpy.array([-(L-D)/2, +W/2, 1.0]).T, axis = 1) # The relative or current frame position of the fourth vertex of the current linkage
        # if i == 0:
        #     rot = numpy.append(rot, numpy.array([[[numpy.cos(theta), -numpy.sin(theta), 0], [numpy.sin(theta), numpy.cos(theta), 0], [0.0, 0.0, 1.0]]]), axis = 0)
        # else:
        #     rot = numpy.append(rot, numpy.array([[[numpy.cos(theta), -numpy.sin(theta), D], [numpy.sin(theta), numpy.cos(theta), 0], [0.0, 0.0, 1.0]]]), axis = 0)
        # Applying successive transformations for the current link as number as the current linkage index (stored in "rot")
        for j in range(1, len(rot)):
            # The transformations should begin from the last one (the last relative transformation matrix), so we should multiply by "rot[-j]" instead of "rot[j]"
            joint = numpy.matmul(rot[-j], joint) # At the end of all successive transformations, its value is the absolute or world frame position of the joint.
            v_1 = numpy.matmul(rot[-j], v_1)
            v_2 = numpy.matmul(rot[-j], v_2)
            v_3 = numpy.matmul(rot[-j], v_3)
            v_4 = numpy.matmul(rot[-j], v_4)

        # Appending the absolute position of the current joint to "joint_positions"
        joint_positions.append(joint.flatten()[:-1].tolist())
        # Appending the absolute positions of the current linkage vertices to "link_vertices"
        link_vertices.append([v_1.flatten()[:-1].tolist(), v_2.flatten()[:-1].tolist(), v_3.flatten()[:-1].tolist(), v_4.flatten()[:-1].tolist()])
        # print(joint_positions) # For debugging!
        # print(link_vertices) # For debugging!

    return (joint_positions, link_vertices)
    # raise NotImplementedError


if __name__ == "__main__":
    chain = get_chain_msg()
    plot_chain(chain.config, chain.W, chain.L, chain.D)
