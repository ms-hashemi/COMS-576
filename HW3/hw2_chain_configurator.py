#!/usr/bin/env python
import sys, argparse
import rospy


def publish(config, W, L, D):
    """Publish the configuration of a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link
    """
    # TODO: Implement this function
    # Done similar to the talker node in Lab5!
    from cs476.msg import Chain2D
    pub = rospy.Publisher('chain_config', Chain2D, queue_size=1) # queue_size=1 means there is only one message buffered
    rospy.init_node('talker', anonymous=True) # Initializing the talker node
    rate = rospy.Rate(1) # Frequency of broadcasting the message by the initialized talker node is 1 Hz.
    while not rospy.is_shutdown():
        msg_config = [config[i] for i in range(len(config))]
        msg_W = W
        msg_L = L
        msg_D = D
        pub.publish(msg_config, msg_W, msg_L, msg_D) # Publishing the message according to "Chain2D.msg" format
        rate.sleep()
    # raise NotImplementedError


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Display the arrangement of 2D kinematic chain"
    )
    parser.add_argument(
        "config",
        metavar="config",
        type=float,
        nargs="+",
        help="chain configuration theta_1, ..., theta_m",
    )
    parser.add_argument(
        "-W", type=float, required=True, dest="W", help="the width of each link"
    )
    parser.add_argument(
        "-L", type=float, required=True, dest="L", help="the length of each link"
    )
    parser.add_argument(
        "-D",
        type=float,
        required=True,
        dest="D",
        help="the distance between the two points of attachment",
    )

    args = parser.parse_args(sys.argv[1:])

    return args


if __name__ == "__main__":
    args = parse_args()

    try:
        publish(args.config, args.W, args.L, args.D)
    except rospy.ROSInterruptException:
        pass
