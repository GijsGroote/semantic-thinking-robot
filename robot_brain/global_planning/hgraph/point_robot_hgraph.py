import numpy as np
from pyvis.network import Network
from robot_brain.global_planning.hgraph.hgraph import HGraph
from robot_brain.global_planning.conf_set_node import ConfSetNode
from robot_brain.global_planning.object_set_node import ObjectSetNode
from robot_brain.global_planning.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.global_variables import FIG_BG_COLOR

from casadi import vertcat
from robot_brain.controller.mpc.mpc import Mpc
from robot_brain.global_planning.hgraph.local_planning.graph_based.circular_robot_occupancy_map import (
    CircleRobotOccupancyMap,
)
from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.conf_set_node import ConfSetNode
from robot_brain.global_planning.object_set_node import ObjectSetNode
from robot_brain.global_planning.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.global_planning.edge import Edge
import math
from robot_brain.state import State

class PointRobotHGraph(HGraph):
    """
    Hypothesis graph for a Point Robot.
    """
    def __init__(self, robot):
        HGraph.__init__(self)

        print("the pointrobot is now created")
        
        self.robot = robot
    
    def estimate_robot_path_existance(self, target_state, objects):

        occ_graph = CircleRobotOccupancyMap(1, 10, 12, objects, self.robot.state.get_xy_position(), 1, 0.8)
        
        # temp fix for negative angles
        start = self.robot.state.get_2d_pose()
        if self.robot.state.get_2d_pose()[2] < 0:
            start[2] = self.robot.state.get_2d_pose()[2]+2*math.pi

        occ_graph.setup()
        self.path = occ_graph.shortest_path(start, target_state.get_2d_pose())

    def robot(self):
        # TODO: sanitize and make private
        return self.robot
