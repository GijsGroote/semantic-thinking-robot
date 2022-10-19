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

        occ_graph = CircleRobotOccupancyMap(cell_size=0.5,
                grid_x_length= 10,
                grid_y_length= 12,
                objects= objects,
                robot_cart_2d= self.robot.state.get_xy_position(),
                robot_radius= 0.4)
        

        occ_graph.setup()
        occ_graph.visualise()
        return occ_graph.shortest_path(self.robot.state.get_xy_position(), target_state.get_xy_position())

    def create_mpc_driving_controller(self):


        print('creating an mpc controller from inside the pointrobot thingy')
        controller = Mpc()
        # dyn_model = Dynamics()
        # dyn_model.set_boxer_model()
        def dyn_model(x, u):
            dx_next = vertcat(
                x[0] + 0.05 *  u[0],
                x[1] + 0.05 *  u[1],
                x[2],
            )
            return dx_next

        controller.setup(dyn_model, self.robot.state, self.robot.state)
        
        return controller


    def robot(self):
        # TODO: sanitize and make private
        return self.robot
