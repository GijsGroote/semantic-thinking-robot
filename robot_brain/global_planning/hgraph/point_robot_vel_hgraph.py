import math
import torch
import numpy as np
from casadi import vertcat

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.sphere_obstacle import SphereObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.obstacle import Obstacle
from robot_brain.global_planning.hgraph.hgraph import HGraph
from robot_brain.global_variables import DT, TORCH_DEVICE, GRID_X_SIZE, GRID_Y_SIZE, POINT_ROBOT_RADIUS
from robot_brain.state import State
from robot_brain.controller.controller import Controller
from robot_brain.controller.drive.mpc.mpc_2th_order import DriveMpc2thOrder
from robot_brain.controller.drive.mppi.mppi_2th_order import DriveMppi2thOrder
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_obstacle_path_estimator import CircleObstaclePathEstimator
from robot_brain.global_planning.hgraph.local_planning.graph_based.rectangle_obstacle_path_estimator import RectangleObstaclePathEstimator
from robot_brain.global_planning.hgraph.local_planning.graph_based.path_estimator import PathEstimator
from robot_brain.controller.push.mppi.mppi_5th_order import PushMppi5thOrder
from robot_brain.system_model import SystemModel
from robot_brain.global_planning.hgraph.local_planning.sample_based.motion_planner import MotionPlanner
from robot_brain.global_planning.hgraph.local_planning.sample_based.drive_motion_planner import DriveMotionPlanner
from robot_brain.global_planning.hgraph.local_planning.sample_based.push_motion_planner import PushMotionPlanner
from robot_brain.controller.push.push_controller import PushController
from robot_brain.controller.drive.drive_controller import DriveController

from helper_functions.geometrics import which_side_point_to_line, circle_in_box_obstacle, circle_in_cylinder_obstacle


DRIVE_MPC_MODEL= "drive_mpc_model"
DRIVE_MPPI_MODEL= "drive_mppi_model"
PUSH_MPPI_MODEL = "push_mppi_model"
PUSH_MPPI_MODEL2 = "push_mppi_model2"

class PointRobotVelHGraph(HGraph):
    """
    Hypothesis graph for a Point Robot accepting velocity input.
    """
    def __init__(self, robot, env):
        HGraph.__init__(self, env)
        self.robot = robot
        self.robot_order = 2

    def create_drive_path_estimator(self, obstacles) -> PathEstimator:
        occ_graph = CircleObstaclePathEstimator(
                cell_size=0.1,
                grid_x_length = GRID_X_SIZE,
                grid_y_length = GRID_Y_SIZE,
                obstacles= obstacles,
                obst_cart_2d= self.robot.state.get_xy_position(),
                obst_name = self.robot.name,
                obst_radius= POINT_ROBOT_RADIUS)

        occ_graph.setup()

        return occ_graph

    def create_push_path_estimator(self, push_obstacle, obstacles):

        if isinstance(push_obstacle.properties, BoxObstacle):

            occ_graph = RectangleObstaclePathEstimator(
                    cell_size=0.1,
                    grid_x_length = GRID_X_SIZE,
                    grid_y_length = GRID_Y_SIZE,
                    obstacles= obstacles,
                    obst_cart_2d= push_obstacle.state.get_xy_position(),
                    obst_name = push_obstacle.name,
                    n_orientations= 3,
                    single_orientation=True,
                    obst_x_length= push_obstacle.properties.length(),
                    obst_y_length= push_obstacle.properties.width())


        elif isinstance(push_obstacle.properties, (CylinderObstacle, SphereObstacle)):
            occ_graph = CircleObstaclePathEstimator(cell_size=0.1,
                    grid_x_length= GRID_X_SIZE,
                    grid_y_length= GRID_Y_SIZE,
                    obstacles= obstacles,
                    obst_cart_2d= self.robot.state.get_xy_position(),
                    obst_name = push_obstacle.name,
                    obst_radius= push_obstacle.properties.radius())
        else:
            raise ValueError("Unknown obstacle encountered during estimating a path")

        occ_graph.setup()

        return occ_graph

    def create_drive_motion_planner(self, obstacles, path_estimator=None) -> DriveMotionPlanner:
        return DriveMotionPlanner(
                grid_x_length=GRID_X_SIZE,
                grid_y_length=GRID_Y_SIZE,
                obstacle=self.robot,
                step_size=0.2,
                search_size=0.4,
                path_estimator=path_estimator)

    def create_push_motion_planner(self, obstacles, push_obstacle, path_estimator=None) -> PushMotionPlanner:
        if isinstance(push_obstacle.properties, BoxObstacle):
            include_orien = True
        elif isinstance(push_obstacle.properties, (CylinderObstacle, SphereObstacle)):
            include_orien = False
        else:
            raise ValueError("Unknown obstacle encountered during estimating a path")

        return PushMotionPlanner(
                grid_x_length=GRID_X_SIZE,
                grid_y_length=GRID_Y_SIZE,
                obstacle=push_obstacle,
                step_size=0.2,
                search_size=0.4,
                path_estimator=path_estimator,
                include_orien=include_orien)

    def in_obstacle(self, pose_2ds) -> list:
        """ return the obstacle keys at pose_2ds. """

        obst_keys = []
        for pose_2d in pose_2ds:
            xy_pos = np.array([pose_2d[0], pose_2d[1]])
            for obst in self.obstacles.values():
                if isinstance(obst.properties, BoxObstacle):
                    if circle_in_box_obstacle(xy_pos, obst, POINT_ROBOT_RADIUS):
                        obst_keys.append(obst.name)

                elif isinstance(obst.properties, CylinderObstacle):
                    if circle_in_cylinder_obstacle(xy_pos, obst, POINT_ROBOT_RADIUS):
                        obst_keys.append(obst.name)

                else:
                    raise ValueError(f"obstacle unknown: {type(obst)}")

        obst_keys = [*set(obst_keys)]

        return obst_keys

    def find_push_pose_againts_obstacle_state(self, blocking_obst, path, path_estimator) -> State:
        """ return a starting state to start pushing the obstacle. """

        print(' in find_push_pose_againts_obstacle_state aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')

        obst_xy_position = path[0][0:2]

        if len(path)>4:
            clost_obst_xy_position = path[3][0:2]
        else:
            raise ValueError(f"path is shorter than 4 samples, its: {len(path)}")

        # retrieve min and max dimension of the obstacle
        if isinstance(blocking_obst.properties, BoxObstacle):
            min_obst_dimension = min(blocking_obst.properties.width(), blocking_obst.properties.length())
            max_obst_dimension = max(blocking_obst.properties.width(), blocking_obst.properties.length())
        elif isinstance(blocking_obst.properties, CylinderObstacle):
            min_obst_dimension = max_obst_dimension = blocking_obst.properties.radius()
        else:
            raise ValueError(f"obstacle not recognised, type: {type(blocking_obst)}")

        dxdy = -(obst_xy_position[0]-clost_obst_xy_position[0])/(obst_xy_position[1]-clost_obst_xy_position[1])



        # for obst_center_to_push_position in np.linspace(min_obst_dimension, 2*max_obst_dimension, 11):
        #
        #     temp_xy_position = [obst_xy_position[0] + np.cos(dxdy)*obst_center_to_push_position,
        #             obst_xy_position[1] + np.sin(dxdy) * obst_center_to_push_position]
        #     if path_estimator.occupancy(temp_xy_position) == 0:
        #         return State(pos=np.array([*temp_xy_position, 0]))

        # for xy_pos in xy_positions:

        # check if opposite of the trajectory direction is free space

        # check if close to opposite of the trajectory is free space


        # check if there is any free space around the obstacle

        return State(pos=np.array([-0.9, 0.1, 0.0]))

    def find_free_state_for_blocking_obstacle(self, blocking_obst: Obstacle, path: list) -> State:
        """ return a state where the obstacle can be pushed toward so it is not blocking the path. """
        print('in find_free_state_for_blocking_obstacle AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')



        return State(pos=np.array([-4, 0.2, -0.1]))



    def get_drive_controllers(self) -> list:
        """ returns list with all possible driving controllers. """

        return [self.create_mppi_drive_controller(),
                self.create_mpc_drive_controller()]

    def find_compatible_models(self, controllers: list) -> list:
        """ find every compatible model for every drive controller. """

        controllers_and_models = []

        # find every compatible model
        for controller in controllers:
            models = []

            if isinstance(controller, DriveMpc2thOrder):
                models.append(DRIVE_MPC_MODEL)

            if isinstance(controller, DriveMppi2thOrder):
                models.append(DRIVE_MPPI_MODEL)

            if isinstance(controller, PushMppi5thOrder):
                models.append(PUSH_MPPI_MODEL)
                models.append(PUSH_MPPI_MODEL2)

            controllers_and_models.append((controller, models))

        return controllers_and_models

    def create_drive_model(self, model_name: str) -> SystemModel:
        """ return the corresponding drive model. """

        if model_name == DRIVE_MPC_MODEL:
            return self.create_mpc_drive_model()
        elif model_name == DRIVE_MPPI_MODEL:
            return self.create_mppi_drive_model()
        else:
            raise ValueError(f"controller name unknown: {model_name}")

    def get_push_controllers(self) -> list:
        return [self.create_mppi_push_controller()]

    def create_push_model(self, model_name: str):
        if model_name == PUSH_MPPI_MODEL:
            return self.create_mppi_push_model()
        elif model_name == PUSH_MPPI_MODEL2:
            return self.create_mppi_push_model2()
        else:
            raise ValueError(f"model name unknown: {model_name}")


    def setup_drive_controller(self, controller, system_model):

        assert isinstance(controller, DriveController), f"the controller should be an DriveController and is {type(controller)}"
        controller.setup(system_model, self.robot.state, self.robot.state)

    def setup_push_controller(self, controller, system_model, push_edge):

        assert isinstance(controller, PushController), f"the controller should be an PushController and is {type(controller)}"

        source_state = self.get_node(push_edge.source).obstacle.state
        controller.setup(system_model, self.robot.state, source_state, source_state)


    ##### DRIVE MPPI #####
    def create_mppi_drive_controller(self):
        """ create MPPI controller for driving an point robot velocity. """
        return DriveMppi2thOrder()


    ##### DRIVE MPC #####
    def create_mpc_drive_controller(self):
        return DriveMpc2thOrder()

    def create_mpc_drive_model(self):

        def model(x, u):
            dx_next = vertcat(
                x[0] + 0.05 *  u[0],
                x[1] + 0.05 *  u[1]
            )
            return dx_next

        return SystemModel(model, name=DRIVE_MPC_MODEL)

    def create_mppi_drive_model(self):

        def model(x, u):

            x_next = torch.zeros(x.shape, dtype=torch.float64, device=TORCH_DEVICE)
            x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT)
            x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT)

            return x_next

        return SystemModel(model, name=DRIVE_MPPI_MODEL)

    ##### PUSH MPPI #####
    def create_mppi_push_controller(self):
        """ create MPPI push controller. """
        return PushMppi5thOrder()

    def create_mppi_push_model(self):
        """ create push model for MPPI pointrobot. """
        def model(x, u):

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

        return SystemModel(model, PUSH_MPPI_MODEL)

    def create_mppi_push_model2(self):
        """ create push model for MPPI pointrobot. """
        def model(x, u):

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

        return SystemModel(model, PUSH_MPPI_MODEL2)
