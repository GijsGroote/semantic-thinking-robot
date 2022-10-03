import pytest
import numpy as np

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle
from motion_planning_env.sphere_obstacle import SphereObstacle

from robot_brain.planning.object import Object
from robot_brain.planning.state import State
from robot_brain.planning.graph_based.circular_robot_occupancy_map import CircleRobotOccupancyMap


def test_occupancy_map_arguements():
    occ_map = CircleRobotOccupancyMap(1, 100, 200, {}, 5)

    assert 1 == occ_map.cell_size
    assert 100 == occ_map.grid_x_length
    assert 200 == occ_map.grid_y_length
    assert 5 == occ_map.robot_radius

def test_grid_map_shape():
    occ_map = CircleRobotOccupancyMap(1, 100, 200, {}, 5)
    assert (100, 200) == occ_map.grid_map.shape


def test_rectange_cylinder_cube_obstacles():

    cylinder_dict = {
            "type": "cylinder",
            "position": [1.0, 1.0, 1.0],
            "geometry": {"radius": 3., "height": 0.3},
            }
    cylinder_object = Object("simple_cylinder", State(pos=np.array([15, 0, 0])), "urdf")
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
    box_object = Object("simple_box", State(pos=np.array([-10,0,0]), ang_p=np.array([0,0,1])), "urdf")

    box_object.obstacle =  BoxObstacle(name="simple_box", content_dict=box_dict)


    objects = {"cylinder": cylinder_object,
            "sphere": sphere_object,
            "box": box_object}

    occ_map = CircleRobotOccupancyMap(1, 40, 60, objects, 2)
    occ_map.setup()
    occ_map.visualise()
    
    assert False
