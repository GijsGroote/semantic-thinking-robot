# import numpy as np
# from robot_brain.global_planning.hgraph.hgraph import HGraph
# from robot_brain.global_variables import FIG_BG_COLOR, DT, TORCH_DEVICE
#
# from casadi import vertcat
# from robot_brain.controller.drive.mpc.mpc_3th_order import DriveMpc3thOrder
# from robot_brain.controller.drive.mppi.mppi_3th_order import DriveMppi3thOrder
# import torch
# from robot_brain.local_planning.graph_based.rectangle_obstacle_path_estimator import (
#     RectangleObstaclePathEstimator,
# )
# from robot_brain.global_planning.kgraph.kgraph import KGraph
# from robot_brain.global_planning.hgraph.object_node import ObjectNode
# from robot_brain.global_planning.kgraph.change_of_state_node import ChangeOfStateNode
# from robot_brain.global_planning.edge import Edge
# import math
# from robot_brain.state import State
#
# class BoxerRobotVelHGraph(HGraph):
#     """
#     Hypothesis graph for a Boxer Robot acceping velocity input.
#     """
#     def __init__(self, robot):
#         HGraph.__init__(self)
#         self.robot = robot
#         self.robot_order = 3
#         
#     
#     def estimate_robot_path_existance(self, target_state, obstacles):
#
#         occ_graph = RectangleObstaclePathEstimator(0.5, 15, 15, obstacles, self.robot.state.get_xy_position(), 4, 1.2, 0.6)
#         
#         # temp fix for negative angles
#         start = self.robot.state.get_2d_pose()
#         occ_graph.setup()
#
#         occ_graph.visualise()
#         path = occ_graph.shortest_path(start, target_state.get_2d_pose())
#         
#         return path
#
#     def get_driving_controllers(self) -> list:
#         """ returns list with all possible driving controllers. """
#
#         # TODO: find banned controllers, find blacklist, ask Kgraph for advice, 
#         # fallback option is random select over all the availeble controllers
#         return [self._create_mpc_driving_controller,
#                 self._create_mppi_driving_controller]
#
#     def get_pushing_controllers(self) -> list:
#         raise NotImplementedError()
#
#     def _create_mppi_driving_controller(self):
#         """ create MPPI controller for driving an boxerc robot velocity. """
#
#         controller = DriveMppi3thOrder()
#
#         def dyn_model(x, u):
#             
#             x_next = torch.zeros(x.shape, dtype=torch.float64, device=TORCH_DEVICE)
#
#             x_next[:,0] = x[:,0] + DT*torch.cos(x[:,2])*u[:,0] 
#             x_next[:,1] = x[:,1] + DT*torch.sin(x[:,2])*u[:,0] 
#             x_next[:,2] = x[:,2] + DT*u[:,1]
#
#             return x_next
#
#         controller.setup(dyn_model=dyn_model,
#                 current_state=self.robot.state,
#                 target_state=self.robot.state)
#
#         return controller
#     def _create_mpc_driving_controller(self):
#
#         controller = DriveMpc3thOrder()
#         # dyn_model = Dynamics()
#         # dyn_model.set_boxer_model()
#         def dyn_model(x, u):
#             dx_next = vertcat(
#                 # you could make DT a bit larger, used to be 0.05
#
#                 x[0] + DT * np.cos(x[2]) * u[0],
#                 x[1] + DT * np.sin(x[2]) * u[0],
#                 x[2] + DT * u[1],
#             )
#             return dx_next
#
#         controller.setup(dyn_model, self.robot.state, self.robot.state)
#         
#         return controller
