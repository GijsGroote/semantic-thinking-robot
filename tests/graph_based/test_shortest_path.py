import pytest
import numpy as np
import math

from robot_brain.global_planning.hgraph.local_planning.graph_based.rectangular_robot_configuration_grid_map import RectangularRobotConfigurationGridMap
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_robot_configuration_grid_map import CircleRobotConfigurationGridMap

from tests.graph_based.obstacle_data.boxes import box
from tests.graph_based.obstacle_data.spheres import sphere
from tests.graph_based.obstacle_data.cylinders import cylinder
from robot_brain.obstacle import Obstacle
from robot_brain.state import State


def test_shortest_path_rect():
    occ_map = RectangularRobotConfigurationGridMap(
            cell_size=1,
            grid_x_length=10,
            grid_y_length=10,
            obstacles={},
            robot_cart_2d=np.array([0,0]),
            n_orientations=8,
            robot_x_length=1,
            robot_y_length=1)
    
    expected_paths = []
    starts = []
    targets = []

    # from middle to south east corner
    expected_paths.append([(-0.1,-0.1,0), (0.5,0.5,0), (1.5,1.5,0), (2.5,2.5,0), (3.3,3.2,0)])
    starts.append((-0.1,-0.1,0))
    targets.append((3.3, 3.2, 0))

     # from north west corner to north east corner
    expected_paths.append([(-4.1,-4.1,0), (-4.5,-3.5,0), (-4.5,-2.5,0), (-4.5,-1.5,0),
        (-4.5,-0.5,0),(-4.5,0.5,0),(-4.5,1.5,0),(-4.5,2.5,0), (-4.5,3.5,0), (-4.3, 4.2, 0)])
    starts.append((-4.1,-4.1,0))
    targets.append((-4.3, 4.2, 0))
    # Turning to 3*pi/4
    expected_paths.append([(0, 0, 2*math.pi-0.2), (0.5, 0.5, math.pi/4), (0.5, 0.5, math.pi/2), (0, 0, 3*math.pi/4+0.03)])
    starts.append((0, 0, -0.2))
    targets.append((0, 0, 3*math.pi/4+0.03))
    
    for (expected_path, start, target) in zip(expected_paths, starts, targets):
        path = occ_map.shortest_path(np.array(start), np.array(target))
        assert path == expected_path

def test_shortest_path_circ():
    occ_map = CircleRobotConfigurationGridMap(
            cell_size=1,
            grid_x_length=10,
            grid_y_length=10,
            obstacles={},
            robot_cart_2d=np.array([0,0]),
            robot_radius=1)
    
    expected_paths = []
    starts = []
    targets = []

    # from middle to south east corner
    expected_paths.append(([(-0.1,-0.1), (0.5,0.5), (1.5,1.5), (2.5,2.5), (3.3,3.2)], True))
    starts.append((-0.1,-0.1))
    targets.append((3.3, 3.2))

     # from north west corner to north east corner
    expected_paths.append(([(-4.1,-4.1), (-4.5,-3.5), (-4.5,-2.5), (-4.5,-1.5),
        (-4.5,-0.5),(-4.5, 0.5),(-4.5,1.5),(-4.5,2.5), (-4.5,3.5), (-4.3, 4.2)], True))
    starts.append((-4.1,-4.1))
    targets.append((-4.3, 4.2 ))
    
    for (expected_path, start, target) in zip(expected_paths, starts, targets):
        path = occ_map.shortest_path(np.array(start), np.array(target))
        assert path == expected_path


def test_shortest_path_with_obstacles():
    obstacles = {}

    obstacles[box.name()] = Obstacle(box.name(), State(pos=np.array([3.0, 0.0, 0.1])), box)
    obstacles[box.name()].type = "unmovable"
    obstacles[sphere.name()] = Obstacle(sphere.name(), State(pos=np.array([1.0, 1.0, 1.0])), sphere)
    obstacles[sphere.name()].type = "unmovable"
    obstacles[cylinder.name()] = Obstacle(cylinder.name(), State(pos=np.array([-1.0, 3.0, 1.0])), cylinder)
    obstacles[cylinder.name()].type = "unmovable"

    occ_map = RectangularRobotConfigurationGridMap(
            cell_size=2,
            grid_x_length=10,
            grid_y_length=10,
            obstacles=obstacles,
            robot_cart_2d=np.array([0,0]),
            n_orientations=8,
            robot_x_length=1,
            robot_y_length=1)

    occ_map.setup()

    expected_paths = []
    starts = []
    targets = []

    # from north west corner to north east corner
    expected_paths.append([(3, -2, 0),
        (2.0, -2.0, 0.0),
        (0.0, 0.0, 0.0),
        (2.0, 2.0, 0.0),
        (3, 2, 0)])

    starts.append((3, -2, 0))
    # targets.append((3, 0, 0))
    targets.append((3, 2, 0))

    for (expected_path, start, target) in zip(expected_paths, starts, targets):
        path = occ_map.shortest_path(np.array(start), np.array(target))
        assert path == expected_path


    assert True
