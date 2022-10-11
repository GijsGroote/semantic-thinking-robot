import pytest
import numpy as np
import math

from robot_brain.global_planning.hgraph.local_planning.graph_based.rectangular_robot_occupancy_map import RectangularRobotOccupancyMap


# THis cannot be called a test TODO: create actual shortest path test
def test_shortest_path():
    occ_map = RectangularRobotOccupancyMap(1, 10, 10, {}, np.array([0,0,0]),
            n_orientations=1, robot_x_length=1, robot_y_length=1)

    print("starting to find that shortest path:")
    path = occ_map.shortest_path(np.array([-4.8, -4.894, np.pi/2-1]), np.array([4.1, 4 ,0]))
    print("printing the path found")
    print(path)
    print(occ_map.occupancy(np.array([-3, 4, 1])))
    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, 0]))
    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, math.pi/2]))
    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, math.pi]))
    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, 1.5*math.pi]))
    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, math.pi*2-0.001]))

    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, math.pi*2]))

    assert True 
    
