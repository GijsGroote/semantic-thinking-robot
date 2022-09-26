import pytest
import numpy as np

from robot_brain.planning.graph_based.rectangular_robot_occupancy_map import RectangularRobotOccupancyMap
from robot_brain.planning.object import Object

from motion_planning_env.box_obstacle import BoxObstacle

def test_occupancy_map_arguements():
    occ_map = RectangularRobotOccupancyMap(1, 100, 200, 10, 6, 360)

    assert 1 == occ_map.cell_size
    assert 100 == occ_map.grid_length
    assert 200 == occ_map.grid_width
    assert 10 == occ_map.robot_length
    assert 6 == occ_map.robot_width
    assert 360 == occ_map.n_orientations

def test_occupancy_exceptions():
    occ_map = RectangularRobotOccupancyMap(1, 100, 200, 10, 6, 360)

    # x_index to large
    with pytest.raises(ValueError):
        occ_map.occupancy(101, 0, 0)

    # x_index to small
    with pytest.raises(ValueError):
        occ_map.occupancy(-1, 0, 0)

    # y_index to large
    with pytest.raises(ValueError):
        occ_map.occupancy(0, 201, 0)

    # y_index to small
    with pytest.raises(ValueError):
        occ_map.occupancy(0, -10, 0)

    # n_orientations to large
    with pytest.raises(ValueError):
        occ_map.occupancy(40, 80, 400)

    # n_orientations to small
    with pytest.raises(ValueError):
        occ_map.occupancy(40, 80, -40)

    # to many arguements
    with pytest.raises(TypeError):
        occ_map.occupancy(40, 80, 200, 10)

    # boolean instead of int
    with pytest.raises(TypeError):
        occ_map.occupancy(40, 80, True)

def test_setup():
    occ_map = RectangularRobotOccupancyMap(1, 100, 200, 10, 6, 360)

    box_dict = {
        "movable": True,
        "orientation": [1,1,1,1],
        "mass": 3,
        "type": "box",
        "color": [0/255, 255/255, 0/255, 1],
        "position": [2.0, 2.0, 1.0],
        "geometry": {"length": 0.5, "width": 0.4, "height": 0.3},
    }

    box_obstacle = BoxObstacle(name="simple_box", content_dict=box_dict)

    box_object = Object("simple_box", "state", "urdf")
    box_object.obstacle = box_obstacle

    objects = {"simple_box": box_object}

    occ_map.setup(objects)

    assert False
