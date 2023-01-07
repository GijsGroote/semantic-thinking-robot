import torch
import math

from casadi import vertcat

from helper_functions.geometrics import which_side_point_to_line 

from robot_brain.global_planning.hgraph.hgraph import HGraph

from robot_brain.global_variables import DT, TORCH_DEVICE

from robot_brain.state import State

from robot_brain.controller.controller import Controller
from robot_brain.controller.drive.mpc.mpc_2th_order import DriveMpc2thOrder
from robot_brain.controller.drive.mppi.mppi_2th_order import DriveMppi2thOrder
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_obstacle_configuration_grid_map import CircleObstacleConfigurationGridMap
from robot_brain.global_planning.hgraph.local_planning.graph_based.rectangle_obstacle_configuration_grid_map import RectangleObstacleConfigurationGridMap
from robot_brain.global_planning.hgraph.local_planning.graph_based.configuration_grid_map import ConfigurationGridMap
from robot_brain.controller.push.mppi.mppi_5th_order import PushMppi5thOrder
from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.sphere_obstacle import SphereObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.global_planning.hgraph.local_planning.sample_based.motion_planner import MotionPlanner
from robot_brain.global_planning.hgraph.local_planning.sample_based.drive_motion_planner import DriveMotionPlanner
from robot_brain.global_planning.hgraph.local_planning.sample_based.push_motion_planner import PushMotionPlanner
from robot_brain.controller.push.push_controller import PushController
from robot_brain.controller.drive.drive_controller import DriveController



class PointRobotVelHGraph(HGraph):
    """
    Hypothesis graph for a Point Robot accepting velocity input.
    """
    def __init__(self, robot, env):
        HGraph.__init__(self, env)
        self.robot = robot
        self.robot_order = 2
    
    def create_drive_path_estimator(self, obstacles) -> ConfigurationGridMap:
        occ_graph = CircleObstacleConfigurationGridMap(cell_size=0.1,
                grid_x_length= 10,
                grid_y_length= 12,
                obstacles= obstacles,
                obst_cart_2d= self.robot.state.get_xy_position(),
                obst_name = self.robot.name,
                obst_radius= 0.4)

        occ_graph.setup()

        return occ_graph

    def create_push_path_estimator(self, push_obstacle, obstacles):

        if isinstance(push_obstacle.properties, BoxObstacle):

            occ_graph = RectangleObstacleConfigurationGridMap(cell_size=0.5,
                    grid_x_length= 10,
                    grid_y_length= 12,
                    obstacles= obstacles,
                    obst_cart_2d= push_obstacle.state.get_xy_position(),
                    obst_name = push_obstacle.name,
                    n_orientations= 10,
                    obst_x_length= push_obstacle.properties.length(),
                    obst_y_length= push_obstacle.properties.width())


        elif isinstance(push_obstacle.properties, (CylinderObstacle, SphereObstacle)):
            occ_graph = CircleObstacleConfigurationGridMap(cell_size=0.5,
                    grid_x_length= 10,
                    grid_y_length= 12,
                    obstacles= obstacles,
                    obst_cart_2d= self.robot.state.get_xy_position(),
                    obst_name = push_obstacle.name,
                    obst_radius= 0.4)
        else:
            raise ValueError("Unknown obstacle encountered during estimating a path")

        occ_graph.setup()

        return occ_graph

    def create_drive_motion_planner(self, obstacles, path_estimator=None) -> DriveMotionPlanner:
        return DriveMotionPlanner(grid_x_length=10,
                grid_y_length=10,
                obstacles=obstacles,
                obstacle=self.robot,
                step_size=0.5,
                search_size=0.7,
                configuration_grid_map=path_estimator)

    def create_push_motion_planner(self, obstacles, push_obstacle, path_estimator=None) -> PushMotionPlanner:
        if isinstance(push_obstacle.properties, BoxObstacle):
            include_orien = True
        elif isinstance(push_obstacle.properties, (CylinderObstacle, SphereObstacle)):
            include_orien = False
        else:
            raise ValueError("Unknown obstacle encountered during estimating a path")

        return PushMotionPlanner(grid_x_length=10,
                grid_y_length=10,
                obstacles=obstacles,
                obstacle=push_obstacle,
                step_size=0.5,
                search_size=0.7,
                include_orien=include_orien,
                configuration_grid_map=path_estimator)

    def get_drive_controllers(self) -> list:
        """ returns list with all possible driving controllers. """

        return [self._create_mppi_drive_controller,
                self._create_mpc_drive_controller]

    def create_drive_model(self, controller_name: str):
        match controller_name:
            case "MPC":
                return self._create_mpc_drive_model()
            case "MPPI":
                return self._create_mppi_drive_model()
            case _:
                raise ValueError(f"controller name unknown: {controller_name}")

    def get_push_controllers(self) -> list:
        return [self._create_mppi_push_controller]

    def create_push_model(self, controller_name: str):
        match controller_name:
            case "MPPI":
                return self._create_mppi_push_model()
            case _:
                raise ValueError(f"controller name unknown: {controller_name}")


    def _setup_drive_controller(self, controller, dyn_model):

        assert isinstance(controller, DriveController), f"the controller should be an DriveController and is {type(controller)}"
        assert callable(dyn_model), "the dyn_model should be callable function"

        controller.setup(dyn_model, self.robot.state, self.robot.state)

    def _setup_push_controller(self, controller, dyn_model, push_edge):
        # TODO: should the target state not also be passed insead of the robot state?

        assert isinstance(controller, PushController), f"the controller should be an PushController and is {type(controller)}"
        assert callable(dyn_model), "the dyn_model should be callable function"

        source_state = self.get_node(push_edge.source).obstacle.state

        controller.setup(dyn_model, self.robot.state, source_state, source_state)



    ##### DRIVE MPPI #####
    def _create_mppi_drive_controller(self):
        """ create MPPI controller for driving an point robot velocity. """
        return DriveMppi2thOrder()

    def _create_mppi_drive_model(self):
        def dyn_model(x, u):
            x_next = torch.zeros(x.shape, dtype=torch.float64, device=TORCH_DEVICE)
            x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
            x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]
            return x_next
        return dyn_model

    ##### DRIVE MPC #####
    def _create_mpc_drive_controller(self):
        return DriveMpc2thOrder()

    def _create_mpc_drive_model(self):
        def dyn_model(x, u):
            dx_next = vertcat(
                x[0] + 0.05 *  u[0],
                x[1] + 0.05 *  u[1]
            )
            return dx_next
        return dyn_model


    ##### PUSH MPPI #####
    def _create_mppi_push_controller(self):
        """ create MPPI push controller. """
        return PushMppi5thOrder()

    def _create_mppi_push_model(self):
        """ create push model for MPPI pointrobot. """
        def dyn_model(x, u):

            # width square obstacle
            H = 2

            # point a in the center of the pushed against edge
            xa = x[:,2]+torch.sin(x[:,4])*H/2
            ya = x[:,3]-torch.cos(x[:,4])*H/2

            # line parallel to the obstacle edge being pushed against
            a_abc = torch.tan(math.pi/2-x[:,4])
            b_abc = x[:,2]+H/2*torch.sin(x[:,4])-torch.tan(math.pi/2-x[:,4])*(x[:,3]-H/2*torch.cos(x[:,4]))


            # line from center robot to center obstacle
            a_ro = (x[:,0]-x[:,2])/(x[:,1]-x[:,3])
            b_ro = x[:,2]-(x[:,0]-x[:,2])/(x[:,1]-x[:,3])*x[:,3]

            yb = (b_ro-b_abc)/(a_abc-a_ro)
            xb = a_abc*yb+b_abc

            # st is the distance from pushing point to the centerline of the obstacle perpendicular to the edge which is pushed against
            st=1.2*torch.sqrt((xa-xb)**2+(ya-yb)**2)

            # obstacle rotating clockwise (positive) or anti-clockwise (negative)
            st = which_side_point_to_line(
                    x[:,2:4],
                    torch.stack((xa, ya), dim=-1),
                    torch.stack((xb, yb), dim=-1))*st

            # velocity of robot perpendicular to box at point p
            vp = u[:,0]*torch.sin(x[:,4]) + u[:,1]*torch.cos(x[:,4])


            x_next = torch.zeros(x.shape, dtype=torch.float64, device=TORCH_DEVICE)
            x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
            x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]
            x_next[:,2] = torch.add(x[:,2], torch.sin(x[:,4])*(1-torch.abs(2*st/H))*vp, alpha=DT)
            x_next[:,3] = torch.add(x[:,3], torch.cos(x[:,4])*(1-torch.abs(2*st/H))*vp, alpha=DT)
            x_next[:,4] = torch.add(x[:,4], 2*vp/H*st, alpha=DT)

            return x_next

        return dyn_model
