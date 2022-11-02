import numpy as np
import torch
from robot_brain.global_planning.hgraph.hgraph import HGraph
from robot_brain.global_variables import (
        FIG_BG_COLOR,
        DT,
        )

from casadi import vertcat
from robot_brain.controller.mpc.mpc_6th_order import Mpc6thOrder
from robot_brain.controller.mppi.mppi_6th_order import Mppi6thOrder
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
        self.robot_order = 6
        
    
    def estimate_robot_path_existance(self, target_state, obstacles):

        occ_graph = RectangularRobotConfigurationGridMap(0.5, 15, 15, obstacles, self.robot.state.get_xy_position(), 4, 1.2, 0.6)
        
        # temp fix for negative angles
        start = self.robot.state.get_2d_pose()
        occ_graph.setup()

        occ_graph.visualise()
        path = occ_graph.shortest_path(start, target_state.get_2d_pose())
        
        return path

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

        controller = Mppi6thOrder()

        def dyn_model(x, u):
            x_next = torch.zeros(x.shape, dtype=torch.float64, device=torch.device("cpu"))

            x_next[:,0] = x[:,0] + DT*torch.cos(x[:,2]) * x[:,3] # x_next[0] = x_pos + DT * cos(orient) * x_vel
            x_next[:,1] = x[:,1] + DT*torch.sin(x[:,2]) * x[:,3] # x_next[0] = x_pos + DT * cos(orient) * x_vel
            x_next[:,1] = torch.add(x[:,1], x[:,4], alpha=DT) # x_next[1] = x[1] + DT*u[1]
            x_next[:,2] = torch.add(x[:,2], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
            x_next[:,3] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
            x_next[:,4] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]
            x_next[:,5] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]

            return x_next

        controller.setup(dyn_model=dyn_model,
                current_state=self.robot.state,
                target_state=self.robot.state)

        return controller

    def _create_mpc_driving_controller(self):

        controller = Mpc6thOrder()

        def dyn_model(x, u):
            dx_next = vertcat(
                x[0] + DT*x[3],# + 0.5*DT*DT*np.cos(x[2])*u[0], # x_next[0] = x_pos + DT * cos(orient) * x_vel
                x[1] + DT*x[4],# + 0.5*DT*DT*np.sin(x[2])*u[0], # x_next[0] = x_pos + DT * cos(orient) * x_vel
                x[2] + DT*x[5],# + 500*0.5*DT*DT*u[1], # x_next[1] = x[1] + DT*u[1]
                x[3] + DT*np.cos(x[2])*u[0], # x_next[0] = x[0] + DT*u[0]
                x[4] + DT*np.sin(x[2])*u[0], # x_next[0] = x[0] + DT*u[0]
                x[5] + 100*DT*u[1], # x_next[1] = x[1] + DT*u[1]
            )
            return dx_next

        controller.setup(dyn_model, self.robot.state, self.robot.state)
        
        return controller


