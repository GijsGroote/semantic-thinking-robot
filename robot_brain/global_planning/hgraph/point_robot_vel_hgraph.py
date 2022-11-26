import torch
from robot_brain.global_planning.hgraph.hgraph import HGraph

from casadi import vertcat
from robot_brain.controller.controller import Controller
from robot_brain.controller.mpc.mpc_2th_order import Mpc2thOrder
from robot_brain.controller.mppi.mppi_2th_order import Mppi2thOrder
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_robot_configuration_grid_map import CircleRobotConfigurationGridMap
from robot_brain.global_planning.hgraph.local_planning.graph_based.configuration_grid_map import ConfigurationGridMap

from robot_brain.global_planning.hgraph.local_planning.sample_based.motion_planner import MotionPlanner
from robot_brain.global_planning.hgraph.local_planning.sample_based.drive_motion_planner import DriveMotionPlanner

from robot_brain.global_variables import DT


class PointRobotVelHGraph(HGraph):
    """
    Hypothesis graph for a Point Robot accepting velocity input.
    """
    def __init__(self, robot):
        HGraph.__init__(self)
        self.robot = robot
        self.robot_order = 2 
    
    def create_drive_path_estimator(self, obstacles) -> ConfigurationGridMap:
        occ_graph = CircleRobotConfigurationGridMap(cell_size=0.1,
                grid_x_length= 10,
                grid_y_length= 12,
                obstacles= obstacles,
                robot_cart_2d= self.robot.state.get_xy_position(),
                robot_radius= 0.4)

        occ_graph.setup()

        return occ_graph

    def create_push_path_estimator(self, obstacles):
        pass

    def create_drive_motion_planner(self, obstacles) -> MotionPlanner:

        return DriveMotionPlanner(grid_x_length=10,
                grid_y_length=10,
                obstacles=obstacles,
                obstacle=self.robot,
                step_size=0.5,
                search_size=0.7)

    def create_push_motion_planner(self, obstacles):
        pass


    # def estimate_robot_path_existance(self, target_state, obstacles):
    #
    #     occ_graph = CircleRobotConfigurationGridMap(cell_size=0.1,
    #             grid_x_length= 10,
    #             grid_y_length= 12,
    #             obstacles= obstacles,
    #             robot_cart_2d= self.robot.state.get_xy_position(),
    #             robot_radius= 0.4)
    #
    #     occ_graph.setup()
    #     occ_graph.visualise()
    #     return occ_graph.shortest_path(self.robot.state.get_xy_position(), target_state.get_xy_position())

    # def estimate_obstacle_path_existance(self, target_state, obstacles):
    #     raise NotImplementedError()

    # def create_motion_planner(self, obstacles):
    #     """ perform motion planning for driving action. """
    #
    #     return DriveMotionPlanner(grid_x_length=10,
    #             grid_y_length=10,
    #             obstacles=obstacles,
    #             obstacle=self.robot,
    #             step_size=0.5,
    #             search_size=0.7)
    #
    # def search_push_path(self, push_obstacle, target_state, obstacles):
    #     """ perform motion planning for pushing action. """
    #     raise NotImplementedError()

    def get_driving_controllers(self) -> list:
        """ returns list with all possible driving controllers. """

        return [self._create_mppi_driving_controller,
                self._create_mpc_driving_controller]

    def create_driving_model(self, controller_name: str):
        match controller_name:
            case "MPC":
                return self._create_mpc_driving_model()
            case "MPPI":
                return self._create_mppi_driving_model()
            case _:
                raise ValueError(f"controller name unknown: {controller_name}")

    def get_pushing_controllers(self) -> list:
        raise NotImplementedError()

    def create_pushing_model(self, controller_name: str):
        raise NotImplementedError()
    
    def _create_mppi_driving_controller(self):
        """ create MPPI controller for driving an point robot velocity. """
        return Mppi2thOrder()

    def _create_mppi_driving_model(self):
        def dyn_model(x, u):
            x_next = torch.zeros(x.shape, dtype=torch.float64, device=torch.device("cpu"))
            x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
            x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]
            return x_next
        return dyn_model

    def _create_mpc_driving_controller(self):
        return Mpc2thOrder()

    def _create_mpc_driving_model(self):
        def dyn_model(x, u):
            dx_next = vertcat(
                x[0] + 0.05 *  u[0],
                x[1] + 0.05 *  u[1]
            )
            return dx_next
        return dyn_model

    def _setup_driving_controller(self, controller, dyn_model):
        # TODO: should the target state not also be passed insead of the robot state?

        print("setup the drivnig controller")
        assert isinstance(controller, Controller), f"the controller should be an Controller and is {type(controller)}"
        assert callable(dyn_model), "the dyn_model should be callable function"

        controller.setup(dyn_model, self.robot.state, self.robot.state)

