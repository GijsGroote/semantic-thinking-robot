import torch
import numpy as np
import math
from robot_brain.global_planning.hgraph.hgraph import HGraph
from robot_brain.global_planning.obstacle_node import ObstacleNode
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.global_variables import (
        FIG_BG_COLOR,
        DT
        )

from casadi import vertcat
from robot_brain.controller.mpc.mpc import Mpc
from robot_brain.controller.mppi.mppi import Mppi
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_robot_configuration_grid_map import (
    CircleRobotConfigurationGridMap,
)
from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.edge import Edge
from robot_brain.state import State

class PointRobotAccHGraph(HGraph):
    """
    Hypothesis graph for a Point Robot accepting acceleration input.
    """
    def __init__(self, robot):
        HGraph.__init__(self)
        self.robot = robot
        self.robot_order = 4
    
    def estimate_robot_path_existance(self, target_state, obstacles):

        occ_graph = CircleRobotConfigurationGridMap(cell_size=0.5,
                grid_x_length= 10,
                grid_y_length= 12,
                obstacles= obstacles,
                robot_cart_2d= self.robot.state.get_xy_position(),
                robot_radius= 0.4)
        

        occ_graph.setup()
        occ_graph.visualise()
        return occ_graph.shortest_path(self.robot.state.get_xy_position(), target_state.get_xy_position())

    def get_driving_controllers(self) -> list:
        """ returns list with all possible driving controllers. """

        # TODO: find banned controllers, find blacklist, ask Kgraph for advice, 
        # fallback option is random select over all the availeble controllers
        return [self._create_mpc_driving_controller,
                self._create_mppi_driving_controller]

    def get_pushing_controllers(self) -> list:
        raise NotImplementedError()

    def _create_mppi_driving_controller(self):
        """ create MPPI controller for driving an point robot velocity. """

        controller = Mppi(order=self.robot_order)

        def dyn_model(x, u):
            
            x_next = torch.zeros(x.shape, dtype=torch.float64, device=torch.device("cpu"))

            x_next[:,0] = x[:,0] + 5*DT*x[:,2] + 0.5*DT*u[:,0]*u[:,0] # x_pos_next = x_pos + DT * x_vel
            x_next[:,1] = x[:,1] + 5*DT*x[:,3] + 0.5*DT*u[:,0]*u[:,1] # y_pos_next = y_pos + DT * y_vel
            x_next[:,2] = x[:,2] + 4*DT*u[:,0] # x_vel_next = x_vel + DT * acc_x
            x_next[:,3] = x[:,3] + 4*DT*u[:,1] # y_vel_next = y_vel + DT * acc_y

            return x_next

        controller.setup(dyn_model=dyn_model,
                current_state=self.robot.state,
                target_state=self.robot.state)

        return controller

    def _create_mpc_driving_controller(self):

        controller = Mpc(order=self.robot_order)
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


