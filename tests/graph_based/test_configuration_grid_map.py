import pytest
import warnings
import numpy as np
from robot_brain.local_planning.graph_based.rectangle_obstacle_path_estimator import RectangleObstaclePathEstimator

from robot_brain.local_planning.graph_based.circle_obstacle_path_estimator import CircleObstaclePathEstimator

from robot_brain.local_planning.graph_based.path_estimator import PathEstimator

from robot_brain.object import Object, UNMOVABLE, UNKNOWN, MOVABLE
from tests.graph_based.obstacle_data.boxes import box
from robot_brain.state import State


def test_start_warning_not_raised():
    " shortest_path should not raise a warning. """

    circle_conf_grid_map = CircleObstaclePathEstimator(
            cell_size=1,
            grid_x_length=10,
            grid_y_length=10,
            objects={},
            obst_cart_2d=np.array([0,0]),
            obst_name="robot",
            obst_radius=1)

    with warnings.catch_warnings():

        warnings.simplefilter("error")
        circle_conf_grid_map.search_path(np.array([0,0]), np.array([-3,4]))

    rect_conf_grid_map = RectangleObstaclePathEstimator(
            cell_size=1,
            grid_x_length=10,
            grid_y_length=10,
            objects={},
            obst_cart_2d=np.array([0,0]),
            obst_name="robot",
            n_orientations=8,
            obst_x_length=1,
            obst_y_length=1)

    with warnings.catch_warnings():
        warnings.simplefilter("error")
        rect_conf_grid_map.search_path(np.array([0,0,0]), np.array([-3,4,0]))



def test_target_warning_raised():
    " shortest_path should raise a warning for the target state. """
    # box on target pose
    box_obst = Object(box.name(), State(pos=np.array([-3.0, 4.0, 0.1])), box)
    box_obst.type = UNKNOWN

    rect_conf_grid_map = RectangleObstaclePathEstimator(
            cell_size=1,
            grid_x_length=10,
            grid_y_length=10,
            objects={box.name(): box_obst},
            obst_cart_2d=np.array([0,0]),
            obst_name="robot",
            n_orientations=8,
            obst_x_length=1,
            obst_y_length=1)

    with pytest.warns(Warning):
       rect_conf_grid_map.search_path(np.array([0,0,0]), np.array([-3,4,0]))

    circle_conf_grid_map = CircleObstaclePathEstimator(
            cell_size=1,
            grid_x_length=10,
            grid_y_length=10,
            objects={box.name(): box_obst},
            obst_cart_2d=np.array([0,0]),
            obst_name="robot",
            obst_radius=1)


    with pytest.warns(Warning):
        circle_conf_grid_map.search_path(np.array([0,0]), np.array([-3,4]))

    # box on start pose
    box_obst = Object(box.name(), State(pos=np.array([.0, 0, 0.1])), box)
    box_obst.type = MOVABLE 

    rect_conf_grid_map = RectangleObstaclePathEstimator(
            cell_size=1,
            grid_x_length=10,
            grid_y_length=10,
            objects={box.name(): box_obst},
            obst_cart_2d=np.array([0,0]),
            obst_name="robot",
            n_orientations=8,
            obst_x_length=1,
            obst_y_length=1)

    with pytest.warns(Warning):
       rect_conf_grid_map.search_path(np.array([0,0,0]), np.array([-3,4,0]))


    # box on start pose 
    circle_conf_grid_map = CircleObstaclePathEstimator(
            cell_size=1,
            grid_x_length=10,
            grid_y_length=10,
            objects={box.name(): box_obst},
            obst_cart_2d=np.array([0,0]),
            obst_name="robot",
            obst_radius=1)

    with pytest.warns(Warning):
        circle_conf_grid_map.search_path(np.array([0,0]), np.array([-3,4]))



# from robot_brain.local_planning.graph_based.circular_robot_occupancy_map import CircleRobotPathEstimator
# def test_occupancy_map_arguements():
#     occ_map = CircleRobotPathEstimator(1, 100, 200, {}, 5)
#
#     assert 1 == occ_map.cell_size
#     assert 100 == occ_map.grid_x_length
#     assert 200 == occ_map.grid_y_length
#
#
# def test_grid_size_not_compatible_with_cell_width():
#
#     with pytest.raises(ValueError):
#         CircleRobotPathEstimator(3, 90, 91, {}, 5)
#
#     with pytest.raises(ValueError):
#         CircleRobotPathEstimator(3, 91, 90, {}, 5)
#
#     with pytest.raises(ValueError):
#         CircleRobotPathEstimator(3, 91, 91, {}, 5)
#
#
#
#


