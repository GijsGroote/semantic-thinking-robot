import numpy as np
import torch
import math
from robot_brain.global_variables import TORCH_DEVICE

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


