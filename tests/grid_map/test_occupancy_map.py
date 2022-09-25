import pytest

from robot_brain.planning.grid_map.circular_robot_occupancy_map import CircleRobotOccupancyMap


def test_occupancy_map_arguements():
    occ_map = CircleRobotOccupancyMap(1, 100, 200, 5)

    assert 1 == occ_map.cell_width
    assert 100 == occ_map.grid_height
    assert 200 == occ_map.grid_width



def test_grid_size_not_compatible_with_cell_width():

    with pytest.raises(ValueError):
        CircleRobotOccupancyMap(3, 90, 91, 5)

    with pytest.raises(ValueError):
        CircleRobotOccupancyMap(3, 91, 90, 5)

    with pytest.raises(ValueError):
        CircleRobotOccupancyMap(3, 91, 91, 5)






