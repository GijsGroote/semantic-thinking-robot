import numpy as np
import math
import pytest


from helper_functions.geometrics import (
        minimal_distance_point_to_line,
        point_in_rectangle,
        do_intersect,
        to_interval_zero_to_two_pi,
        )

def test_do_overlap():
    lines_intesect = [
            (np.array([-1,0]), np.array([1,0]), np.array([0,1]), np.array([0,-1])),
            (np.array([-1,1]), np.array([1,-1]), np.array([-1,-1]), np.array([1,1])),
            (np.array([-1,0]), np.array([0,0]), np.array([0,0]), np.array([1,0])),
            (np.array([0,-1]), np.array([0,0]), np.array([0,0]), np.array([0,1])),
            (np.array([0,0]), np.array([0,0]), np.array([0,0]), np.array([0,0])),
            ]

    for (l1a, l1b, l2a, l2b) in lines_intesect:
        assert do_intersect(l1a, l1b, l2a, l2b)

    lines_do_not_intesect = [
            (np.array([-100,0]), np.array([100,0]), np.array([-100,0.01]), np.array([100,0.01])),
            (np.array([0,-100]), np.array([0,100]), np.array([0.01,-100]), np.array([0.01,100])),
            (np.array([0,0]), np.array([0,1]), np.array([-100,2]), np.array([100,2])),
            (np.array([0,0]), np.array([1,0]), np.array([2,-100]), np.array([2,100])),
            (np.array([0,0]), np.array([1,0]), np.array([2,0]), np.array([3,0])),
            ]

    for (l1a, l1b, l2a, l2b) in lines_do_not_intesect:
        assert not do_intersect(l1a, l1b, l2a, l2b)

def test_minimal_distance_point_to_line():
    data = [
            (3.0, np.array([3,0]), np.array([0,-2]), np.array([0,2])),
            (3.0, np.array([-3,0]), np.array([0,-2]), np.array([0,2])),
            (3.0, np.array([0,3]), np.array([-2,0]), np.array([2,0])),
            (3.0, np.array([0,-3]), np.array([-2,0]), np.array([2,0])),
            (np.sqrt(2), np.array([1,1]), np.array([-2,0]), np.array([0,0])),
            (np.sqrt(2), np.array([1,-1]), np.array([-2,0]), np.array([0,0])),
            (np.sqrt(2), np.array([-1,1]), np.array([2,0]), np.array([0,0])),
            (np.sqrt(2), np.array([-1,-1]), np.array([2,0]), np.array([0,0])),
            ]

    for (answer, p, lp1, lp2) in data:
        assert answer == pytest.approx(minimal_distance_point_to_line(p, lp1, lp2))

def test_to_zero_to_two_pi():
    assert to_interval_zero_to_two_pi(-math.pi-1) == math.pi-1
    assert to_interval_zero_to_two_pi(0) == 0
    assert to_interval_zero_to_two_pi(math.pi) == math.pi
    assert to_interval_zero_to_two_pi(2*math.pi) == 0
    assert to_interval_zero_to_two_pi(3*math.pi) == math.pi
