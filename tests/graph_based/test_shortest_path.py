import pytest
import numpy as np
import math

from robot_brain.planning.graph_based.rectangular_robot_occupancy_map import RectangularRobotOccupancyMap



def test_shortest_path():
    occ_map = RectangularRobotOccupancyMap(1, 10, 10, {}, np.array([0,0,0]), 2, 1, 1)
   
    print("starting to find that shortest path:")
    occ_map.shortest_path(np.array([-3.2, 4.394, np.pi/2]), np.array([3, -4 ,0]))
    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, 0]))
    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, math.pi/2]))
    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, math.pi]))
    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, 1.5*math.pi]))
    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, math.pi*2-0.001]))

    # occ_map.pose_2d_to_cell_idx(np.array([-2, -2, math.pi*2]))
            

    assert False
    
