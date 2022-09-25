import pytest
import numpy as np

from robot_brain.planning.grid_map.circular_robot_occupancy_map import CircleRobotOccupancyMap


def test_occupancy_map_arguements():
    occ_map = CircleRobotOccupancyMap(1, 100, 200, 5)

    assert 1 == occ_map.cell_width
    assert 100 == occ_map.grid_height
    assert 200 == occ_map.grid_width
    assert 5 == occ_map.robot_radius

def test_grid_map_shape():

    occ_map = CircleRobotOccupancyMap(1, 100, 200, 5)
    assert (200, 100) == occ_map.grid_map.shape
