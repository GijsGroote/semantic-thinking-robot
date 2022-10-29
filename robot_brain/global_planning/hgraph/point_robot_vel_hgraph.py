import torch
from robot_brain.global_planning.hgraph.hgraph import HGraph

from casadi import vertcat
from robot_brain.controller.mpc.mpc import Mpc
from robot_brain.controller.mppi.mppi import Mppi
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_robot_configuration_grid_map import (
    CircleRobotConfigurationGridMap,
)

from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.edge import Edge
from robot_brain.state import State
from robot_brain.global_variables import DT


class PointRobotVelHGraph(HGraph):
    """
    Hypothesis graph for a Point Robot accepting velocity input.
    """
    def __init__(self, robot):
        HGraph.__init__(self)
        self.robot = robot
        self.robot_order = 2 
    
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

        return [self._create_mppi_driving_controller,
                self._create_mpc_driving_controller]

    def get_pushing_controllers(self) -> list:
        raise NotImplementedError()

    def _create_mppi_driving_controller(self):
        """ create MPPI controller for driving an point robot velocity. """

        controller = Mppi(order=self.robot_order)

        def dyn_model(x, u):
            x_next = torch.zeros(x.shape, dtype=torch.float64, device=torch.device("cpu"))

            x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
            x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]

            return x_next

        controller.setup(dyn_model=dyn_model,
                current_state=self.robot.state,
                target_state=self.robot.state)

        return controller


    def _create_mpc_driving_controller(self):
        controller = Mpc(order=self.robot_order)

        def dyn_model(x, u):
            dx_next = vertcat(
                x[0] + 0.05 *  u[0],
                x[1] + 0.05 *  u[1],
                x[2],
            )
            return dx_next

        # TODO: should the target state not also be passed insead of the robot state?
        controller.setup(dyn_model, self.robot.state, self.robot.state)
        return controller
