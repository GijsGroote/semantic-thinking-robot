import math
import numpy as np

from robot_brain.object import Object, UNMOVABLE
from robot_brain.state import State
from robot_brain.local_planning.graph_based.rectangle_object_path_estimator import RectangleObjectPathEstimator
from robot_brain.local_planning.graph_based.circle_object_path_estimator import CircleObjectPathEstimator

from tests.graph_based.obstacle_data.boxes import box
from tests.graph_based.obstacle_data.spheres import sphere
from tests.graph_based.obstacle_data.cylinders import cylinder

def test_shortest_path_rect():
    occ_map = RectangleObjectPathEstimator(
            cell_size=1,
            grid_x_length=10,
            grid_y_length=10,
            objects={},
            obst_cart_2d=np.array([0,0]),
            obst_name="robot",
            n_orientations=8,
            obst_x_length=1,
            obst_y_length=1)

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
        path = occ_map.search_path(np.array(start), np.array(target))
        assert path == expected_path

def test_shortest_path_circ():
    occ_map = CircleObjectPathEstimator(
            cell_size=1,
            grid_x_length=10,
            grid_y_length=10,
            objects={},
            obst_cart_2d=np.array([0,0]),
            obst_name="robot",
            obst_radius=1)

    expected_paths = []
    starts = []
    targets = []

    # from middle to south east corner
    expected_paths.append([(-0.1,-0.1), (0.5,0.5), (1.5,1.5), (2.5,2.5), (3.3,3.2)])
    starts.append((-0.1,-0.1))
    targets.append((3.3, 3.2))

     # from north west corner to north east corner
    expected_paths.append([(-4.1,-4.1), (-4.5,-3.5), (-4.5,-2.5), (-4.5,-1.5),
        (-4.5,-0.5),(-4.5, 0.5),(-4.5,1.5),(-4.5,2.5), (-4.5,3.5), (-4.3, 4.2)])
    starts.append((-4.1,-4.1))
    targets.append((-4.3, 4.2 ))

    for (expected_path, start, target) in zip(expected_paths, starts, targets):
        path = occ_map.search_path(np.array(start), np.array(target))
        assert path == expected_path


def test_shortest_path_with_objects():
    objects = {}

    objects[box.name()] = Object(box.name(), State(pos=np.array([3.0, 0.0, 0.1])), box)
    objects[box.name()].type = UNMOVABLE

    objects[cylinder.name()] = Object(cylinder.name(), State(pos=np.array([-1.0, 3.0, 1.0])), cylinder)
    objects[cylinder.name()].type = UNMOVABLE

    occ_map = RectangleObjectPathEstimator(
            cell_size=2,
            grid_x_length=10,
            grid_y_length=10,
            objects=objects,
            obst_cart_2d=np.array([0,0]),
            obst_name="robot",
            n_orientations=8,
            obst_x_length=1,
            obst_y_length=1)

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
        path = occ_map.search_path(np.array(start), np.array(target))
        assert path == expected_path


    assert True
