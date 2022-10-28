import numpy as np
from robot_brain.global_planning.hgraph.hgraph import HGraph
from robot_brain.global_variables import FIG_BG_COLOR

from casadi import vertcat
from robot_brain.controller.mpc.mpc import Mpc
from robot_brain.global_planning.hgraph.local_planning.graph_based.rectangular_robot_configuration_grid_map import (
    RectangularRobotConfigurationGridMap,
)
from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.obstacle_node import ObstacleNode
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.global_planning.edge import Edge
import math
from robot_brain.state import State

class BoxerRobotAccHGraph(HGraph):
    """
    Hypothesis graph for a Boxer Robot accepting acceleration input.
    """
    def __init__(self, robot):
        HGraph.__init__(self)
        self.robot = robot
        
    
    def estimate_robot_path_existance(self, target_state, obstacles):

        occ_graph = RectangularRobotConfigurationGridMap(0.5, 15, 15, obstacles, self.robot.state.get_xy_position(), 4, 1.2, 0.6)
        
        # temp fix for negative angles
        start = self.robot.state.get_2d_pose()
        occ_graph.setup()

        occ_graph.visualise()
        path = occ_graph.shortest_path(start, target_state.get_2d_pose())
        
        return path

    def create_mpc_driving_controller(self):

        controller = Mpc()
        # dyn_model = Dynamics()
        # dyn_model.set_boxer_model()
        # TODO: MPC should have an accelaration input robot
        def dyn_model(x, u):
            dx_next = vertcat(
                x[0] + 0.025 * np.cos(x[2]) * u[0],
                x[1] + 0.025 * np.sin(x[2]) * u[0],
                x[2] + 0.025 * u[1],
            )
            return dx_next

        controller.setup(dyn_model, self.robot.state, self.robot.state)
        
        return controller


    def robot(self):
        # TODO: sanitize and make private
        return self.robot
