import pytest
import os
import math
import numpy as np

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle
from motion_planning_env.sphere_obstacle import SphereObstacle
from motion_planning_env.urdf_obstacle import UrdfObstacle

from robot_brain.planning.graph_based.rectangular_robot_occupancy_map import RectangularRobotOccupancyMap
from robot_brain.planning.object import Object
from robot_brain.planning.state import State

@pytest.fixture
def objects_set1():

    cylinder_dict = {
            "type": "cylinder",
            "position": [1.0, 1.0, 1.0],
            "geometry": {"radius": 0.5, "height": 0.3},
            }
    cylinder_obstacle = CylinderObstacle(name="simple_cylinder", content_dict=cylinder_dict)
    cylinder_state = State(pos=np.array([5,5,0]))
    cylinder_object = Object("simple_cylinder", cylinder_state, "urdf")
    cylinder_object.obstacle = cylinder_obstacle

    sphere_dict = {
            "type": "sphere",
            "position": [1.0, 1.0, 1.0],
            "geometry": {"radius": 0.5},
            }
    sphere_obstacle = SphereObstacle(name="simple_sphere", content_dict=sphere_dict)
    sphere_state = State(pos=np.array([8,1,0]))
    sphere_object = Object("simple_sphere", sphere_state, "urdf")
    sphere_object.obstacle = sphere_obstacle

    box_dict = {
            "type": "box",
            "position": [2.0, 2.0, 1.0],
            "geometry": {"length": 2, "width": 1, "height": 0.3},
            }
    box_obstacle = BoxObstacle(name="simple_box", content_dict=box_dict)
    box_state = State(pos=np.array([1,1,0]), vel=np.array([1,0,2]))
    box_object = Object("simple_box", box_state, "urdf")
    box_object.obstacle = box_obstacle

    objects = {"simple_box": box_object,
            "simple_cylinder": cylinder_object,
            "simple_sphere": sphere_object}

    return objects

def test_occupancy_map_arguements():
    occ_map = RectangularRobotOccupancyMap(1.0, 100, 200, {}, robot_position=np.array([1,2]), n_orientations=360, robot_x_length=10, robot_y_length=6)

    assert 1 == occ_map.cell_size
    assert 100 == occ_map.grid_x_length
    assert 200 == occ_map.grid_y_length
    assert 10 == occ_map.robot_x_length
    assert 6 == occ_map.robot_y_length
    assert 360 == occ_map.n_orientations

def test_occupancy_exceptions():
    occ_map = RectangularRobotOccupancyMap(1.0, 100, 200, {}, robot_position=np.array([1,2]), n_orientations=360, robot_x_length=10, robot_y_length=6)
    # x_index to large
    with pytest.raises(ValueError):
        occ_map.idx_to_occupancy(101, 0, 0)

    # x_index to small
    with pytest.raises(ValueError):
        occ_map.idx_to_occupancy(-1, 0, 0)

    # y_index to large
    with pytest.raises(ValueError):
        occ_map.idx_to_occupancy(0, 201, 0)

    # y_index to small
    with pytest.raises(ValueError):
        occ_map.idx_to_occupancy(0, -10, 0)

    # n_orientations to large
    with pytest.raises(ValueError):
        occ_map.idx_to_occupancy(40, 80, 400)

    # n_orientations to small
    with pytest.raises(ValueError):
        occ_map.idx_to_occupancy(40, 40, -40)

    # boolean instead of int
    with pytest.raises(TypeError):
        occ_map.idx_to_occupancy(40, 80, True)

def test_cell_idx_to_position_even():
    # test even grid_x_length and grid_y_length
    occ_map = RectangularRobotOccupancyMap(0.5, 4, 4, {}, robot_position=np.array([1,2]), n_orientations=1, robot_x_length=2, robot_y_length=1)
    assert (-1/4, 1/4) == occ_map.cell_idx_to_position(3, 4)
    assert (-1.75, -1.75) == occ_map.cell_idx_to_position(0, 0)
    assert (3/4, 1/4) == occ_map.cell_idx_to_position(5, 4)

def test_cell_idx_to_position_uneven():
    # test uneven grid_x_length and grid_y_length
    occ_map = RectangularRobotOccupancyMap(1.0, 3, 3, {}, robot_position=np.array([1,2]), n_orientations=1, robot_x_length=1, robot_y_length=2)
    assert (-1, -1) == occ_map.cell_idx_to_position(0, 0)
    assert (0, 0) == occ_map.cell_idx_to_position(1, 1)
    assert (0, 1) == occ_map.cell_idx_to_position(1, 2)

def test_cell_idx_to_position_out_of_bounds():
    occ_map = RectangularRobotOccupancyMap(1.0, 4, 7, {}, robot_position=np.array([1,2]), n_orientations=1, robot_x_length=1, robot_y_length=2)
    with pytest.raises(IndexError):
        occ_map.cell_idx_to_position(4, 3)
    with pytest.raises(IndexError):
        occ_map.cell_idx_to_position(3, 7)
    with pytest.raises(IndexError):
        occ_map.cell_idx_to_position(0, -1)
    with pytest.raises(IndexError):
        occ_map.cell_idx_to_position(-1, 0)

def test_position_to_cell_idx_in_cell():
    occ_map = RectangularRobotOccupancyMap(30, 90, 90, {}, robot_position=np.array([1,2]), n_orientations=1, robot_x_length=1, robot_y_length=1)
    assert (2, 2) == occ_map.position_to_cell_idx(15.01, 15.01)
    assert (2, 2) == occ_map.position_to_cell_idx(44.99, 15.01)

    assert (1, 1) == occ_map.position_to_cell_idx(-14.99, 14.99)
    assert (1, 1) == occ_map.position_to_cell_idx(-14.99, -14.99)
    assert (1, 1) == occ_map.position_to_cell_idx(14.99, 14.99)
    assert (1, 1) == occ_map.position_to_cell_idx(14.99, -14.99)

    assert (0, 2) == occ_map.position_to_cell_idx(-15.01, 15.01)
    assert (0, 2) == occ_map.position_to_cell_idx(-15.01, 44.99,)
    assert (0, 2) == occ_map.position_to_cell_idx(-44.99, 15.01)
    assert (0, 2) == occ_map.position_to_cell_idx(-44.99, 44.99,)

def test_position_to_cell_idx_on_boundary():
    occ_map = RectangularRobotOccupancyMap(5.0, 30, 40, {}, robot_position=np.array([1,2]), n_orientations=4, robot_x_length=1, robot_y_length=10)

    assert (0,0) == occ_map.position_to_cell_idx(-15, -17.5)
    assert (1,0) == occ_map.position_to_cell_idx(-10, -17.5)
    assert (0,0) == occ_map.position_to_cell_idx(-12.5, -20)
    assert (0,1) == occ_map.position_to_cell_idx(-12.5, -15)
    assert (5,7) == occ_map.position_to_cell_idx(15, 17.5)
    assert (5,7) == occ_map.position_to_cell_idx(10, 17.5)
    assert (5,7) == occ_map.position_to_cell_idx(12.5, 20)
    assert (5,7) == occ_map.position_to_cell_idx(12.5, 15)

def test_position_to_cell_idx_out_of_bounds():
    occ_map = RectangularRobotOccupancyMap(1, 100, 150, {}, robot_position=np.array([1,2]), n_orientations=4, robot_x_length=1, robot_y_length=10)
    with pytest.raises(IndexError):
        occ_map.position_to_cell_idx(50.1, 3)
    with pytest.raises(IndexError):
        occ_map.position_to_cell_idx(-50.3, 3)
    with pytest.raises(IndexError):
        occ_map.position_to_cell_idx(0, -75.32)
    with pytest.raises(IndexError):
        occ_map.position_to_cell_idx(-1, 75.01)

def test_position_to_cell_or_grid_edge():
    occ_map = RectangularRobotOccupancyMap(1.0, 6, 6, {}, robot_position=np.array([1,2]), n_orientations=4, robot_x_length=1, robot_y_length=10)

    assert (0,0) == occ_map.position_to_cell_idx_or_grid_edge(-5, -5)
    assert (5,5) == occ_map.position_to_cell_idx_or_grid_edge(5, 5)
    assert (5,5) == occ_map.position_to_cell_idx_or_grid_edge(3, 3)
    assert (3,4) == occ_map.position_to_cell_idx_or_grid_edge(0.1, 1.543)

# def test_occupancy_map_circular_objects():
#     occ_map = RectangularRobotOccupancyMap(1, 5, 4, 2, 1, 1)
#
#     cylinder_dict = {
#             "type": "cylinder",
#             "position": [1.0, 1.0, 1.0],
#             "geometry": {"radius": 1., "height": 0.3},
#             }
#     cylinder_object = Object("simple_cylinder", State(pos=np.array([0, 0, 0])), "urdf")
#     cylinder_object.obstacle = CylinderObstacle(name="simple_cylinder", content_dict=cylinder_dict)
#
#     sphere_dict= {
#             "type": "sphere",
#             "position": [1.0, 1.0, 1.0],
#             "geometry": {"radius": 1.},
#             }
#     sphere_object = Object("simple_sphere",  State(pos=np.array([2,-1,0])), "urdf")
#     sphere_object.obstacle = SphereObstacle(name="simple_sphere", content_dict=sphere_dict)
#
#     objects = {"cylinder": cylinder_object,
#             "sphere": sphere_object}
#
#     occ_map.setup(objects)
#     print(occ_map.grid_map)
################################################################################################
############### MANUAL MANUAL MANUAL INSPECTION TESTS, SHOULD BE COMMENTED OUT##################
################################################################################################
# def test_rectange_small():
#     occ_map = RectangularRobotOccupancyMap(0.1, 6, 10, 3, 1, 4)
#
#     box_dict = {
#             "type": "box",
#             "position": [1.0, 1.0, 1.0],
#             "geometry": {"length": 3, "width": 1, "height": 0.3},
#             }
#
#     box_object = Object("simple_box", State(pos=np.array([0,0,0]), ang_p=np.array([0,0,np.pi/2])), "urdf")
#     box_object.obstacle =  BoxObstacle(name="simple_box", content_dict=box_dict)
#
#     objects = {"box": box_object}
#
#     occ_map.setup(objects)
#     print(occ_map.grid_map[:,:,0])
#     occ_map.visualise(nn0, objects)
#     occ_map.visualise(1, objects)
#     occ_map.visualise(2, objects)
#     occ_map.visualise(3, objects)
#
#     assert False
#
#
# def test_rectange_please():
#     occ_map = RectangularRobotOccupancyMap(1, 5, 6, 1, 2, 8)
#
#     box_dict = {
#             "type": "box",
#             "position": [1.0, 1.0, 1.0],
#             "geometry": {"length": 2, "width": 1., "height": 0.3},
#             }
#
#     box_object = Object("simple_box", State(pos=np.array([0,0,0])), "urdf")
#
#     box_object.obstacle =  BoxObstacle(name="simple_box", content_dict=box_dict)
#
#
#     objects = {"box": box_object}
#
#     occ_map.setup(objects)
#
#     print(occ_map.grid_map[:,:,0])
#     occ_map.visualise(0, objects)
#     occ_map.visualise(1, objects)
#     occ_map.visualise(2, objects)
#     occ_map.visualise(3, objects)
#
#     assert False
#
#
def test_rectange_cylinder_cube_obstacles():
        
    urdf_duck_dict= {
        "type": "urdf",
        "position": [2, 0, 0.25],
        "orientation": [math.pi/2, 0, 0],
        "geometry": {
            "urdf": os.path.join(os.path.dirname(__file__), "./obstacle_data/duck/duck.urdf"),
        }
    }
    urdf_object = Object("duck", State(pos=np.array([-15, 0, 0])), "urdf")
    urdf_object.obstacle = UrdfObstacle(name="duck", content_dict=urdf_duck_dict)
    
    cylinder_dict = {
            "type": "cylinder",
            "position": [1.0, 1.0, 1.0],
            "geometry": {"radius": 3., "height": 0.3},
            }
    cylinder_object = Object("simple_cylinder", State(pos=np.array([15, 0, 0]), ang_p=np.array([math.pi/2, 0.5, 0,5])), "urdf")
    cylinder_object.obstacle = CylinderObstacle(name="simple_cylinder", content_dict=cylinder_dict)

    sphere_dict= {
            "type": "sphere",
            "position": [1.0, 1.0, 1.0],
            "geometry": {"radius": 2.},
            }
    sphere_object = Object("simple_sphere",  State(pos=np.array([-5,-10,0])), "urdf")
    sphere_object.obstacle = SphereObstacle(name="simple_sphere", content_dict=sphere_dict)

    box_dict = {
            "type": "box",
            "position": [1.0, 1.0, 1.0],
            "geometry": {"width": 3, "length": 8, "height": 0.3},
            }
    box_object = Object("simple_box", State(pos=np.array([-10,0,0]), ang_p=np.array([0,0,0])), "urdf")

    box_object.obstacle =  BoxObstacle(name="simple_box", content_dict=box_dict)


    objects = {"cylinder": cylinder_object,
            "sphere": sphere_object,
            "duck_urdf": urdf_object,
            "box": box_object}

    occ_map = RectangularRobotOccupancyMap(1.0, 40, 60, objects, robot_position=np.array([1,2]), n_orientations=1, robot_x_length=1, robot_y_length=5)
    occ_map.setup()

    print("printint the shortest path")
    print(occ_map.shortest_path(np.array([0,0,0]), np.array([-10,-15,0])))
    occ_map.visualise(save=False)

    # occ_map.visualise(1, objects)
    # occ_map.visualise(2, objects)
    # occ_map.visualise(3, objects)

    assert False
