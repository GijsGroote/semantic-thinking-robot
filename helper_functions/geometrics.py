import math
import numpy as np
import torch


from robot_brain.global_variables import TORCH_DEVICE, GRID_X_SIZE, GRID_Y_SIZE


# TODO: give proper description to this entire file + functions

def in_grid(x:float, y:float) -> bool:
    """ return True if the (x,y) position in
    inside the grid boundaries, otherwise False. """
    return abs(x)<=GRID_X_SIZE/2 and abs(y)<=GRID_Y_SIZE


def minimal_distance_point_to_line(p: np.ndarray, lp1: np.ndarray, lp2: np.ndarray) -> float:
    """ returns the minimal distance from a point to a line.
    if the normal distance is not on the line, the distance to
    minimal distance to the edge of the line is returned.
    """
    assert p.shape == (2,), "points has an incorrect shape"
    assert lp1.shape == (2,) and lp2.shape == (2,), "line points have incorrect shape"

    dp = lp2 - lp1
    st = dp[0]**2 + dp[1]**2
    u = ((p[0] - lp1[0]) * dp[0] + (p[1] - lp1[1]) * dp[1]) / st

    if u > 1:
        u = 1
    if u < 0:
        u = 0

    dx = (lp1[0] + u * dp[0]) - p[0]
    dy = (lp1[1] + u * dp[1]) - p[1]

    return np.sqrt(dx**2 + dy**2)

def point_in_rectangle(p: np.ndarray, lp1: np.ndarray, lp2: np.ndarray, lp3:np.ndarray) -> bool:
    """ returns true if the point p is inside the rectangle
    which has edges lp1 to lp2 and lp2 to lp3.
    """
    if isinstance(p, tuple):
        p = np.array(p)
    assert p.shape == (2,), "points has an incorrect shape"
    assert (lp1.shape == (2,) and
    lp2.shape == (2,) and lp3.shape == (2,)), "line points have incorrect shape"

    dp1 = lp2 - lp1
    st1 = dp1[0]**2 + dp1[1]**2
    u1 = ((p[0] - lp1[0]) * dp1[0] + (p[1] - lp1[1]) * dp1[1]) / st1

    dp2 = lp2 - lp3
    st2 = dp2[0]**2 + dp2[1]**2
    u2 = ((p[0] - lp3[0]) * dp2[0] + (p[1] - lp3[1]) * dp2[1]) / st2

    return u1 >= 0 and u1 <= 1 and u2 >= 0 and u2 <= 1

def do_intersect(p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray) -> bool:
    """ check if 2 lines segments overlap. """
    # Find the 4 orientations required for
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True

    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
    if ((o1 == 0) and on_segment(p1, p2, q1)):
        return True

    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
    if ((o2 == 0) and on_segment(p1, q2, q1)):
        return True

    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
    if ((o3 == 0) and on_segment(p2, p1, q2)):
        return True

    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
    if ((o4 == 0) and on_segment(p2, q1, q2)):
        return True

    # If none of the cases
    return False

def on_segment(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> bool:
    """
    Given three collinear points p1, p2, p3, the function checks if
    point p2 lies on line segment from p1 to p3.
    """
    if ((q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and
            (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))):
        return True

    return False

def orientation(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> int:
    """
    to find the orientation of an ordered triplet (p,q,r)
    function returns the following values:
    0 : Collinear points
    1 : Clockwise points
    2 : Counterclockwise

    See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
    for details of below formula.
    """
    val = (float(q[1] - p[1]) * (r[0] - q[0])) - (float(q[0] - p[0]) * (r[1] - q[1]))
    if val > 0:
        # Clockwise orientation
        return 1

    elif val < 0:
        # Counterclockwise orientation
        return 2

    else:
        # Collinear orientation
        return 0

def to_interval_zero_to_two_pi(val: float) -> float:
    """ returns the angle in interval [0, 2*pi). """
    while val >= 2*math.pi:
        val -= 2*math.pi
    while val < 0:
        val += 2*math.pi
    return val

def to_interval_min_pi_to_pi(val: float) -> float:
    """ returns the angle in interval [-pi, pi). """
    while val > math.pi:
        val -= 2*math.pi
    while val <= -math.pi:
        val += 2*math.pi
    return val

def which_side_point_to_line(a: torch.Tensor, b: torch.Tensor, p: torch.Tensor) -> torch.Tensor:
    """ find if point p is on right (True) or left (False) side of line from a to b. """

    right_or_left_bool = ((b[:,0]-a[:,0])*(p[:,1]-a[:,1])-(b[:,1]-a[:,1])*(p[:,0]-a[:,0]) < 0)
    right_or_left = torch.ones(a.size(dim=0), device=TORCH_DEVICE)
    right_or_left[right_or_left_bool] = -1

    return right_or_left

def check_floats_divisible(x: float, y: float, scaling_factor: float = 1e4):
    """ check if x / y == 0. """

    scaled_x = int(x * scaling_factor)
    scaled_y = int(y * scaling_factor)

    return (scaled_x % scaled_y) == 0

def circle_in_box_object(xy_pos: np.ndarray, obj, radius: float) -> bool:
    """ Return True if the circle overlaps with the box object. """
    cos_ol = math.cos(obj.state.ang_p[2])*obj.properties.width()/2
    sin_ol = math.sin(obj.state.ang_p[2])*obj.properties.width()/2
    cos_ow = math.cos(obj.state.ang_p[2])*obj.properties.length()/2
    sin_ow = math.sin(obj.state.ang_p[2])*obj.properties.length()/2

    obj_a = np.array([obj.state.pos[0]-sin_ol+cos_ow, obj.state.pos[1]+cos_ol+sin_ow])
    obj_b = np.array([obj.state.pos[0]-sin_ol-cos_ow, obj.state.pos[1]+cos_ol-sin_ow])
    obj_c = np.array([obj.state.pos[0]+sin_ol-cos_ow, obj.state.pos[1]-cos_ol-sin_ow])
    obj_d = np.array([obj.state.pos[0]+sin_ol+cos_ow, obj.state.pos[1]-cos_ol+sin_ow])


    if point_in_rectangle(xy_pos, obj_a, obj_b, obj_c):
        return True

    elif minimal_distance_point_to_line(xy_pos, obj_a, obj_b) <= radius:
        return True

    elif minimal_distance_point_to_line(xy_pos, obj_b, obj_c) <= radius:
        return True

    elif minimal_distance_point_to_line(xy_pos, obj_c, obj_d) <= radius:
        return True

    elif minimal_distance_point_to_line(xy_pos, obj_d, obj_a) <= radius:
        return True

    else:
        return False

def circle_in_cylinder_object(xy_pos, obj, radius: float) -> bool:
    """ Return True if the circle overlaps with the cylinder objacle. """
    return np.linalg.norm(xy_pos-obj.state.get_xy_position()) <= obj.properties.radius() + radius

def box_in_cylinder_object(cylinder_obj, box_obj):
    box_obj_orien = box_obj.state.ang_p[2]

    cos_rl = math.cos(box_obj_orien)*box_obj.properties.width()/2
    sin_rl = math.sin(box_obj_orien)*box_obj.properties.width()/2
    cos_rw = math.cos(box_obj_orien)*box_obj.properties.length()/2
    sin_rw = math.sin(box_obj_orien)*box_obj.properties.length()/2

    box_obj_xy = box_obj.state.get_xy_position()
    cylinder_obj_xy = cylinder_obj.state.get_xy_position()

    if (np.linalg.norm(cylinder_obj.state.get_xy_position()-box_obj_xy) <= \
        cylinder_obj.properties.radius() + min(box_obj.properties.length(), box_obj.properties.width()) / 2):
        return True

    # corner points of the object
    a = box_obj_xy + np.array([-sin_rl+cos_rw, cos_rl+sin_rw])

    b = box_obj_xy + np.array([-sin_rl-cos_rw, cos_rl-sin_rw])

    c = box_obj_xy + np.array([+sin_rl-cos_rw, -cos_rl-sin_rw])

    d = box_obj_xy + np.array([sin_rl+cos_rw, -cos_rl+sin_rw])


    # check if the edges of the objects overlap with the cylinder
    if minimal_distance_point_to_line(cylinder_obj_xy, a, b) <= cylinder_obj.properties.radius():
        return True

    elif minimal_distance_point_to_line(cylinder_obj_xy, b, c) <= cylinder_obj.properties.radius():
        return True

    elif minimal_distance_point_to_line(cylinder_obj_xy, c, d) <= cylinder_obj.properties.radius():
        return True

    elif minimal_distance_point_to_line(cylinder_obj_xy, d, a) <= cylinder_obj.properties.radius():
        return True

    return False

def box_in_box_object(in_this_box_obj, box_obj):

    box_obj_orien = box_obj.state.get_2d_pose()[2]

    cos_rl = math.cos(box_obj_orien)*box_obj.properties.width()/2
    sin_rl = math.sin(box_obj_orien)*box_obj.properties.width()/2
    cos_rw = math.cos(box_obj_orien)*box_obj.properties.length()/2 # x
    sin_rw = math.sin(box_obj_orien)*box_obj.properties.length()/2

    cos_ol = math.cos(in_this_box_obj.state.ang_p[2])*in_this_box_obj.properties.width()/2
    sin_ol = math.sin(in_this_box_obj.state.ang_p[2])*in_this_box_obj.properties.width()/2
    cos_ow = math.cos(in_this_box_obj.state.ang_p[2])*in_this_box_obj.properties.length()/2
    sin_ow = math.sin(in_this_box_obj.state.ang_p[2])*in_this_box_obj.properties.length()/2
    # corner points of the in_this_box_objacle
    obj_a= np.array([in_this_box_obj.state.pos[0]-sin_ol+cos_ow, in_this_box_obj.state.pos[1]+cos_ol+sin_ow])
    obj_b = np.array([in_this_box_obj.state.pos[0]-sin_ol-cos_ow, in_this_box_obj.state.pos[1]+cos_ol-sin_ow])
    obj_c = np.array([in_this_box_obj.state.pos[0]+sin_ol-cos_ow, in_this_box_obj.state.pos[1]-cos_ol-sin_ow])
    obj_d = np.array([in_this_box_obj.state.pos[0]+sin_ol+cos_ow, in_this_box_obj.state.pos[1]-cos_ol+sin_ow])

    box_obj_xy = box_obj.state.get_xy_position()

    if point_in_rectangle(box_obj.state.get_xy_position(), obj_a, obj_b, obj_c):
        return True

   # corner points of the objacle
    a = box_obj_xy + np.array([-sin_rl+cos_rw, cos_rl+sin_rw])

    b = box_obj_xy + np.array([-sin_rl-cos_rw, cos_rl-sin_rw])

    c = box_obj_xy + np.array([+sin_rl-cos_rw, -cos_rl-sin_rw])

    d = box_obj_xy + np.array([sin_rl+cos_rw, -cos_rl+sin_rw])

    if (do_intersect(a, b, obj_a, obj_b) or
        do_intersect(a, b, obj_b, obj_c) or
        do_intersect(a, b, obj_c, obj_d) or
        do_intersect(a, b, obj_d, obj_a) or

        do_intersect(b, c, obj_a, obj_b) or
        do_intersect(b, c, obj_b, obj_c) or
        do_intersect(b, c, obj_c, obj_d) or
        do_intersect(b, c, obj_d, obj_a) or

        do_intersect(c, d, obj_a, obj_b) or
        do_intersect(c, d, obj_b, obj_c) or
        do_intersect(c, d, obj_c, obj_d) or
        do_intersect(c, d, obj_d, obj_a) or

        do_intersect(d, a, obj_a, obj_b) or
        do_intersect(d, a, obj_b, obj_c) or
        do_intersect(d, a, obj_c, obj_d) or
        do_intersect(d, a, obj_d, obj_a)):

        return True

    return False
