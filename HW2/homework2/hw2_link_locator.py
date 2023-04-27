from hw2_chain_plotter import get_link_positions


def is_in_half_space(v, v1, v2):
    """Determine whether v is in the half space to the left of the vector from v1 to v2

    @type v, v1, v2: a tuple (x, y) that indicates the x and y coordinates of the corresponding points.

    """
    x = v[0]
    y = v[1]
    x1 = v1[0]
    y1 = v1[1]
    x2 = v2[0]
    y2 = v2[1]

    a = y2 - y1
    b = x1 - x2
    c = x2 * y1 - x1 * y2

    return a * x + b * y + c <= 0


def is_in_polygon(v, vertices):
    """Determine whether v is in the polygon with the given vertices (assumed to be given in the CC order)

    @type v: a tuple (x, y)
    @type vertices: a list [p1, ..., p_{m+1}] where p_i is the position (x,y) of the i^{th} vertex.
    """

    for i in range(len(vertices)):
        v1 = vertices[i]
        v2 = vertices[0]
        if i + 1 < len(vertices):
            v2 = vertices[i + 1]
        if not is_in_half_space(v, v1, v2):
            return False
    return True


def get_link_indices_containing(v, config, W, L, D):
    """Determine the indices of all the links that contain v

    @type v: a tuple (x, y)
    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link
    """
    (joint_positions, link_vertices) = get_link_positions(config, W, L, D)
    link_indices = []

    for i in range(len(link_vertices)):
        if is_in_polygon(v, link_vertices[i]):
            link_indices.append(i+1)

    return link_indices
