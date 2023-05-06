import math
import sys
import copy
import warnings
import random

from typing import Tuple
import torch
import numpy as np
from casadi import vertcat

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.sphere_obstacle import SphereObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.object import Object, UNMOVABLE, FREE
from robot_brain.global_planning.hgraph.hgraph import HGraph
from robot_brain.global_variables import DT, TORCH_DEVICE, GRID_X_SIZE, GRID_Y_SIZE, POINT_ROBOT_RADIUS
from robot_brain.state import State
from robot_brain.controller.controller import Controller
from robot_brain.controller.drive.mpc.mpc_2th_order import DriveMpc2thOrder
from robot_brain.controller.drive.mppi.mppi_2th_order import DriveMppi2thOrder
from robot_brain.local_planning.graph_based.circle_object_path_estimator import CircleObjectPathEstimator
from robot_brain.local_planning.graph_based.rectangle_object_path_estimator import RectangleObjectPathEstimator
from robot_brain.local_planning.graph_based.path_estimator import PathEstimator
from robot_brain.global_planning.hgraph.push_act_edge import PushActionEdge
from robot_brain.controller.push.mppi.mppi_5th_order import PushMppi5thOrder
from robot_brain.controller.push.mppi.mppi_4th_order import PushMppi4thOrder
from robot_brain.system_model import SystemModel
from robot_brain.local_planning.sample_based.motion_planner import MotionPlanner
from robot_brain.local_planning.sample_based.drive_motion_planner import DriveMotionPlanner
from robot_brain.global_planning.hgraph.drive_act_edge import DriveActionEdge
from robot_brain.local_planning.sample_based.push_motion_planner import PushMotionPlanner
from robot_brain.controller.push.push_controller import PushController
from robot_brain.controller.drive.drive_controller import DriveController

from robot_brain.exceptions import NoBestPushPositionException, NoPathExistsException, NoTargetPositionFoundException

from helper_functions.extract_object_info import get_min_max_dimension_from_object
from helper_functions.geometrics import (
        which_side_point_to_line,
        box_in_cylinder_object,
        box_in_box_object,
        circle_in_box_object,
        circle_in_cylinder_object,
        to_interval_zero_to_two_pi,
        in_grid,
        )


DRIVE_MPC_MODEL= "drive_mpc_model"
DRIVE_MPPI_MODEL= "drive_mppi_model"
PUSH_MPPI_MODEL = "push_mppi_model1"
PUSH_MPPI_MODEL2 = "push_mppi_model2"
PUSH_MPPI_MODEL_4TH_ORDER = "push_mppi_model_4th_order"



class PointRobotVelHGraph(HGraph):
    """
    Hypothesis graph for a Point Robot accepting velocity input.
    """
    def __init__(self, robot_obj: Object):
        HGraph.__init__(self,
                robot_obj=robot_obj,
                robot_order=2)

    def create_drive_path_estimator(self, objects) -> PathEstimator:
        """ create drive path estimator. """
        occ_graph = CircleObjectPathEstimator(
                cell_size=0.1,
                grid_x_length = GRID_X_SIZE,
                grid_y_length = GRID_Y_SIZE,
                objects= objects,
                obst_cart_2d= self.robot_obj.state.get_xy_position(),
                obst_name = self.robot_obj.name,
                obst_radius= POINT_ROBOT_RADIUS)

        occ_graph.setup()

        return occ_graph

    def create_push_path_estimator(self, push_obj: Object, objects: dict ) -> PathEstimator:
        """ create push path estimator. """
        assert isinstance(push_obj, Object), f"push_obj should be Object and is {type(push_obj)}"
        assert isinstance(objects, dict), f"objects should be dict and is {type(objects)}"

        if isinstance(push_obj.properties, BoxObstacle):

            occ_graph = RectangleObjectPathEstimator(
                    cell_size = 0.1,
                    grid_x_length = GRID_X_SIZE,
                    grid_y_length = GRID_Y_SIZE,
                    objects= objects,
                    obst_cart_2d = push_obj.state.get_xy_position(),
                    obst_name = push_obj.name,
                    n_orientations = 3,
                    single_orientation = False,
                    obst_x_length = push_obj.properties.length(),
                    obst_y_length = push_obj.properties.width())


        elif isinstance(push_obj.properties, (CylinderObstacle, SphereObstacle)):
            occ_graph = CircleObjectPathEstimator(cell_size=0.1,
                    grid_x_length= GRID_X_SIZE,
                    grid_y_length= GRID_Y_SIZE,
                    objects= objects,
                    obst_cart_2d= self.robot_obj.state.get_xy_position(),
                    obst_name = push_obj.name,
                    obst_radius= push_obj.properties.radius())
        else:
            raise ValueError("Unknown object encountered during estimating a path")

        occ_graph.setup()

        return occ_graph

    def create_drive_motion_planner(self, objects: dict, path_estimator: PathEstimator):
        """ create drive motion planner. """
        assert isinstance(objects, dict), f"objects should be dict and is {type(objects)}"
        assert isinstance(path_estimator, PathEstimator), f"path_estimator should be PathEstimator and is {type(path_estimator)}"

        return DriveMotionPlanner(
                grid_x_length=GRID_X_SIZE,
                grid_y_length=GRID_Y_SIZE,
                obj=self.robot_obj,
                step_size=0.2,
                search_size=0.35,
                path_estimator=path_estimator)

    def create_push_motion_planner(self, objects: dict, push_obj: Object, path_estimator: PathEstimator) -> PushMotionPlanner:
        """ create push manipulation planner. """
        assert isinstance(objects, dict), f"objects should be dict and is {type(objects)}"
        assert isinstance(push_obj, Object), f"objects should be Object and is {type(push_obj)}"
        assert isinstance(path_estimator, PathEstimator), f"path_estimator should be PathEstimator and is {type(path_estimator)}"

        if isinstance(push_obj.properties, BoxObstacle):
            include_orien = True
        elif isinstance(push_obj.properties, (CylinderObstacle, SphereObstacle)):
            include_orien = False
        else:
            raise ValueError("Unknown object encountered during creation of PushMotionPlanner")

        return PushMotionPlanner(
                grid_x_length=GRID_X_SIZE,
                grid_y_length=GRID_Y_SIZE,
                obj=push_obj,
                step_size=0.2,
                search_size=0.25,
                path_estimator=path_estimator,
                include_orien=include_orien)

    def in_object(self, pose_2ds: list, obj: Object, objects: dict) -> list:
        """ return the object keys at pose_2ds that are in collision with obj. """
        assert isinstance(pose_2ds, list), f"pose_2ds should be a list and is {type(pose_2ds)}"
        for temp_pose_2d in pose_2ds:
            assert isinstance(temp_pose_2d, list), f"temp_pose_2d should be a list and is {type(temp_pose_2d)}"
            for temp in temp_pose_2d:
                assert isinstance(temp, (float, int)),\
                        f"temp_pose_2d should contain only float or int and contains a: {type(temp)}"
        assert isinstance(obj, Object), f"obj should be a Object and is {type(Object)}"
        assert isinstance(objects, dict), f"objects should be a dict and is {type(objects)}"

        blocking_obj_keys = []

        if obj.name == self.robot_obj.name:
            blocking_obj_keys = self.robot_in_object(pose_2ds, objects)
        elif isinstance(obj.properties, BoxObstacle):
            blocking_obj_keys = self.box_obj_in_object(pose_2ds, obj, objects)
        elif isinstance(obj.properties, CylinderObstacle):
            blocking_obj_keys = self.cylinder_obj_in_object(pose_2ds, obj, objects)
        else:
            raise ValueError("unknown object")

        # if nothing is found, return closes object that is not obj
        if len(blocking_obj_keys) == 0:

            for pose_2d in pose_2ds:
                min_dist = sys.maxsize
                closest_obj_key = None

                for temp_obj in objects.values():

                    # exclude the obj
                    if temp_obj.name == obj.name:
                        continue

                    temp_dist = np.linalg.norm(temp_obj.state.get_xy_position() - pose_2d[0:2])

                    if temp_dist < min_dist:
                        min_dist = temp_dist
                        closest_obj_key = temp_obj.name

                if closest_obj_key is not None:
                    blocking_obj_keys.append(closest_obj_key)

        if len(blocking_obj_keys) == 0:
            warnings.warn("could not find object")
            return

        return [*set(blocking_obj_keys)]


    def robot_in_object(self, pose_2ds: list, objects: dict) -> list:
        obj_keys = []
        for pose_2d in pose_2ds:
            xy_pos = np.array([pose_2d[0], pose_2d[1]])
            for obj in objects.values():
                if isinstance(obj.properties, BoxObstacle):
                    if circle_in_box_object(xy_pos, obj, POINT_ROBOT_RADIUS):
                        obj_keys.append(obj.name)

                elif isinstance(obj.properties, CylinderObstacle):
                    if circle_in_cylinder_object(xy_pos, obj, POINT_ROBOT_RADIUS):
                        obj_keys.append(obj.name)

                else:
                    raise ValueError(f"object unknown: {type(obj)}")

        return obj_keys

    def box_obj_in_object(self, pose_2ds: list, obj: Object, objects: dict) -> list:
        blocking_obj_keys = []
        for pose_2d in pose_2ds:

            for temp_obj in objects.values():
                if temp_obj.name == obj.name:
                    continue

                if isinstance(temp_obj.properties, BoxObstacle):
                    if box_in_box_object(temp_obj, obj):
                        blocking_obj_keys.append(temp_obj.name)

                elif isinstance(temp_obj.properties, CylinderObstacle):
                    if box_in_cylinder_object(temp_obj, obj):
                        blocking_obj_keys.append(temp_obj.name)

                else:
                    raise ValueError(f"object unknown: {type(obj)}")

        return blocking_obj_keys


    def cylinder_obj_in_object(self, pose_2ds: list, obj: Object, objects: dict) -> list:
        blocking_obj_keys = []
        for pose_2d in pose_2ds:
            xy_pos = np.array([pose_2d[0], pose_2d[1]])

            for temp_obj in objects.values():
                if temp_obj.name == obj.name:
                    continue

                if isinstance(temp_obj.properties, BoxObstacle):
                    if circle_in_box_object(xy_pos, temp_obj, obj.properties.radius()):
                        blocking_obj_keys.append(temp_obj.name)

                elif isinstance(temp_obj.properties, CylinderObstacle):
                    if circle_in_cylinder_object(xy_pos, temp_obj, obj.properties.radius()):
                        blocking_obj_keys.append(temp_obj.name)

                else:
                    raise ValueError(f"object unknown: {type(obj)}")

        return blocking_obj_keys

    def find_best_push_state_againts_object(self, objects: dict, blocking_obj: Object, path: list) -> State:
        """ return a state to push against blocking_obj to later push blocking_obj over path. """
        assert isinstance(objects, dict)
        assert isinstance(blocking_obj, Object), f"blocking_obj should be Object and is {type(blocking_obj)}"
        assert isinstance(path, list), f"path should be an list and is {type(path)}"
        for temp_list in path:
            assert isinstance(temp_list, list), f"path should contain only tuple and contains a: {type(temp_list)}"
            # TODO: fix this commented stuff
            # for temp in temp_list:
            #     assert isinstance(temp, float), f"temp_type should contain only float an contains a: {type(temp)}"

        obj_xy_position = path[0][0:2]

        if len(path)>4:
            clost_obj_xy_position = path[3][0:2]
        elif len(path)>=2:
            clost_obj_xy_position = path[1][0:2]
        else:
            raise ValueError(f"path is shorter than 2 samples, its: {len(path)}")

        # retrieve min and max dimension of the object
        (min_obj_dimension, max_obj_dimension) = get_min_max_dimension_from_object(blocking_obj)

        # orientation where the robot should stand to push
        if (clost_obj_xy_position[1]-obj_xy_position[1]) == 0:
            if clost_obj_xy_position[0] < obj_xy_position[0]:
                best_robot_pose_orien = math.pi/2
            else:
                best_robot_pose_orien = -math.pi/2
        else:
            best_robot_pose_orien = math.atan2(-(clost_obj_xy_position[0]-obj_xy_position[0]),\
                    clost_obj_xy_position[1]-obj_xy_position[1]) + math.pi

        path_estimator = self.create_drive_path_estimator(objects)

        bo = best_robot_pose_orien
        pi = math.pi
        try_these_orien = [bo, bo-pi/100, bo+pi/100,
                bo-pi/80, bo+pi/80, bo-pi/60, bo+pi/60,
                bo-pi/40, bo+pi/40, bo-pi/20, bo+pi/20,
                bo-pi/10, bo+pi/10, bo-pi/5, bo+pi/5]

        for temp_orien in try_these_orien:
            temp_orien = to_interval_zero_to_two_pi(temp_orien)

            xy_position_in_free_space = self._find_first_xy_position_in_free_space(obj_xy_position,
                    min_obj_dimension, 2*max_obj_dimension, temp_orien, path_estimator)

            if xy_position_in_free_space is not None and in_grid(xy_position_in_free_space[0], xy_position_in_free_space[1]):

                return State(pos=np.array([*xy_position_in_free_space, 0]))

        raise NoBestPushPositionException(f"could not find a push position against object {blocking_obj.name}")

    def find_free_state_for_blocking_object(self, objects: dict, blocking_obj: Object, path: list) -> State:
        """ return a state where the object can be pushed toward so it is not blocking the path. """
        assert isinstance(objects, dict)
        assert isinstance(blocking_obj, Object), f"blocking_obj should be Object and is {type(blocking_obj)}"
        assert isinstance(path, list), f"path should be an list and is {type(path)}"
        for temp_pose_2d in path:
            assert isinstance(temp_pose_2d, list),\
                    f"path should contain lists and contains a: {type(temp_pose_2d)}"
            # TODO: check the lists
            # for temp_tuple in temp_pose_2d:
            #     assert isinstance(temp_tuple, (float, int)),\
            #             "temp_pose_2d should contain only floats and integers"\
            #             f"and contains a: {type(temp_tuple)} {path}"

        # return State(pos=np.array([0, 4, 0]))

        # TODO: works for circlular objects I think, but there are not orientations taken into account now

        # pretend that the object is unmovable
        blocking_obj_type = blocking_obj.type
        blocking_obj.type = UNMOVABLE

        # configuration space for the robot
        path_estimator_robot = self.create_drive_path_estimator(objects)

        # find reachable position around the object
        reachable_xy_positions = []
        (min_obj_dimension, max_obj_dimension) = get_min_max_dimension_from_object(blocking_obj)
        obj_xy_position = blocking_obj.state.get_xy_position()
        blocking_obj_orien = to_interval_zero_to_two_pi(blocking_obj.state.get_2d_pose()[2])

        # find reachable positions around blocking object
        for temp_orien in np.linspace(0, 2*math.pi, 8):
            temp_orien = to_interval_zero_to_two_pi(temp_orien)

            xy_position_in_free_space = self._find_first_xy_position_in_free_space(obj_xy_position,
                    min_obj_dimension, 2*max_obj_dimension, temp_orien, path_estimator_robot)

            if xy_position_in_free_space is not None:
                try:
                    path = path_estimator_robot.search_path(self.robot_obj.state.get_xy_position(), xy_position_in_free_space)
                    reachable_xy_positions.append([len(path), *xy_position_in_free_space])

                except NoPathExistsException:
                    pass

        blocking_obj.type = blocking_obj_type


        if len(reachable_xy_positions) == 0:
            raise NoTargetPositionFoundException(f"no positions where reachable around object {blocking_obj.name}")
        reachable_xy_positions.sort()
        reachable_xy_positions = np.array(reachable_xy_positions).reshape((len(reachable_xy_positions), 3))
        reachable_xy_positions = reachable_xy_positions[:, 1:3].reshape((len(reachable_xy_positions), 2))

        # configuration space for the object
        objects_and_path = copy.deepcopy(objects)

        # This operation takes quite some time.
        for (i, robot_xy_position_in_path) in enumerate(path):
            objects_and_path['path_position_'+str(i)] = Object('path_position_'+str(i),
                    State(pos=np.array([robot_xy_position_in_path[0], robot_xy_position_in_path[1], 0])),
                    self.robot_obj.properties, obj_type=UNMOVABLE)

        path_estimator_obj = self.create_push_path_estimator(blocking_obj, objects_and_path)

        # create the orientations
        obj_target_oriens = []
        for robot_xy_pos in reachable_xy_positions:

            obj_target_oriens.append(math.atan2(robot_xy_pos[0]-obj_xy_position[0],\
                    robot_xy_pos[1]-obj_xy_position[1]))


        # check pushing in a straight line
        for blocking_obj_to_target_dist in np.linspace(max_obj_dimension, 5, 50):
            for temp_orien in obj_target_oriens:
                temp_orien = to_interval_zero_to_two_pi(temp_orien)

                temp_x = float(obj_xy_position[0] - np.sin(temp_orien)*blocking_obj_to_target_dist)
                temp_y = float(obj_xy_position[1] + np.cos(temp_orien)*blocking_obj_to_target_dist)
                temp_2d_pose = np.array([temp_x, temp_y, blocking_obj_orien])


                if in_grid(temp_2d_pose[0], temp_2d_pose[1]) and path_estimator_obj.occupancy(temp_2d_pose) == FREE:

                    return State(pos=np.array([temp_2d_pose[0], temp_2d_pose[1], 0]),
                            ang_p=np.array([0, 0, temp_2d_pose[2]]))

        ot = obj_target_oriens
        pi = math.pi
        obj_target_semi_orien = []

        for temp_orien in obj_target_oriens:
            for orien_addition in [pi/100, pi/80, pi/60, pi/40, pi/20, pi/10, pi/5]:
                obj_target_oriens.append(temp_orien + orien_addition)
                obj_target_oriens.append(temp_orien - orien_addition)

        # check pushing in a tilted line
        for blocking_obj_to_target_dist in np.linspace(max_obj_dimension, 5, 50):
            for temp_orien in obj_target_semi_orien:
                temp_orien = to_interval_zero_to_two_pi(temp_orien)

                temp_x = float(obj_xy_position[0] - np.sin(temp_orien)*blocking_obj_to_target_dist)
                temp_y = float(obj_xy_position[1] + np.cos(temp_orien)*blocking_obj_to_target_dist)
                temp_2d_pose = np.array([temp_x, temp_y, blocking_obj_orien])

                if in_grid(temp_2d_pose[0], temp_2d_pose[1]) and path_estimator_obj.occupancy(temp_2d_pose) == FREE:

                    return State(pos=np.array([temp_2d_pose[0], temp_2d_pose[1], 0]),
                            ang_p=np.array([0, 0, temp_2d_pose[2]]))

        raise NoTargetPositionFoundException


    def _find_first_xy_position_in_free_space(self, obj_xy_position: np.ndarray, dist_from_obj_min: float, dist_from_obj_max: float,
                                    orien: float,  path_estimator: PathEstimator) -> np.ndarray:
        """ return the first pose that is in free space if no
            pose_2d is in free space, nothing is returned.

            obj_xy_position:    objects position is xy coordinates
            dist_from_obj_min:  min distance from object
            dist_from_obj_max:  maximal distance from object
            orien:              orientation between line where points to check lie, and positive y axis.
            path_estimator:     configuration space of the robot

        """
        for xy_pos_temp in np.linspace(dist_from_obj_min, dist_from_obj_max, 50):

            temp_xy_position = [obj_xy_position[0] - np.sin(orien)*xy_pos_temp,
                    obj_xy_position[1] + np.cos(orien) * xy_pos_temp]

            if in_grid(temp_xy_position[0], temp_xy_position[1]) and path_estimator.occupancy(temp_xy_position) == FREE:
                return temp_xy_position

    def get_drive_controllers(self) -> list:
        """ returns list with all possible driving controllers. """
        # return [self.create_mpc_drive_controller()]

        return [self.create_mppi_drive_controller(),
                self.create_mpc_drive_controller()]

    def find_compatible_models(self, controllers: list) -> list:
        """ find every compatible model for every drive controller. """
        assert isinstance(controllers, list), f"controllers should be list and is {type(controllers)}"
        for temp_controller in controllers:
            assert isinstance(temp_controller, Controller), f"controllers should contain only Controller and contains a: {type(temp_controller)}"

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

            if isinstance(controller, PushMppi4thOrder):
                models.append(PUSH_MPPI_MODEL_4TH_ORDER)

            controllers_and_models.append((controller, models))

        return controllers_and_models

    def get_drive_controller_not_in_kgraph(self, target_iden: int, kgraph_suggestions: list) -> Tuple[Controller, str]:
        """ find controller and system model name that is not in kgraph_suggestions. """
        assert isinstance(target_iden, int), f"target_iden should be int and is {type(target_iden)}"
        assert isinstance(kgraph_suggestions, (list, type(None)))

        controllers = self.get_drive_controllers()
        controllers_and_model_names = self.find_compatible_models(controllers)

        # filter the blacklisted edges
        controllers_and_model_names_filtered = self.filter_control_and_model_names(
                DriveActionEdge,
                controllers_and_model_names,
                target_iden)

        controller = None
        model_name = None

        if kgraph_suggestions is None:
            return self.create_drive_controller(target_iden)

        else:
            # check if a controller is not yet in kgraph_suggestions
            for (temp_controller, temp_model_names) in controllers_and_model_names_filtered:
                # TODO: check not only the controller, but also the system model
                in_kgraph = False
                for kgraph_controller in kgraph_suggestions:
                    if isinstance(temp_controller, type(kgraph_controller)):
                        in_kgraph = True
                    else:
                        not_in_kgraph_controller = temp_controller
                        not_in_kgraph_model_names = temp_model_names
                if not in_kgraph:
                    (controller, model_names) = (not_in_kgraph_controller, not_in_kgraph_model_names)
                    model_name = random.choice(model_names)
                    break

        return (controller, model_name)

    def create_drive_controller(self, target_iden: int) -> Tuple[Controller, str]:
        """ randomly select a driving controller that is not on the blacklist. """
        assert isinstance(target_iden, int), f"target_iden should be int and is {type(target_iden)}"

        controllers = self.get_drive_controllers()
        controllers_and_model_names = self.find_compatible_models(controllers)

        # filter the blacklisted edges
        controllers_and_model_names_filtered = self.filter_control_and_model_names(
                DriveActionEdge,
                controllers_and_model_names,
                target_iden)

        (controller, model_names) = random.choice(controllers_and_model_names_filtered)
        model_name = random.choice(model_names)

        return (controller, model_name)

    def get_push_controller_not_in_kgraph(self, target_iden: int, kgraph_suggestions: list) -> Tuple[Controller, str]:
        """ find controller and system model name that is not in kgraph_suggestions. """
        assert isinstance(target_iden, int), f"target_iden should be int and is {type(target_iden)}"
        assert isinstance(kgraph_suggestions, (list, type(None)))

        controllers = self.get_push_controllers()
        controllers_and_model_names = self.find_compatible_models(controllers)

        # filter the blacklisted edges
        controllers_and_model_names_filtered = self.filter_control_and_model_names(
                PushActionEdge,
                controllers_and_model_names,
                target_iden)

        controller = None
        model_name = None

        if kgraph_suggestions is None:
            return self.create_push_controller(target_iden)

        else:
            # check if a controller is not yet in kgraph_suggestions
            for (temp_controller, temp_model_names) in controllers_and_model_names_filtered:
                # TODO: check not only the controller, but also the system model
                in_kgraph = False
                for kgraph_controller in kgraph_suggestions:
                    if isinstance(temp_controller, type(kgraph_controller)):
                        in_kgraph = True
                    else:
                        not_in_kgraph_controller = temp_controller
                        not_in_kgraph_model_names = temp_model_names
                if not in_kgraph:
                    (controller, model_names) = (not_in_kgraph_controller, not_in_kgraph_model_names)
                    model_name = random.choice(model_names)
                    break

        return (controller, model_name)



    def create_push_controller(self, target_iden: int) -> Tuple[Controller, str]:
        """ create push controller. """
        assert isinstance(target_iden, int), f"target_iden should be int and is {type(target_iden)}"

        controllers = self.get_push_controllers()
        controllers_and_model_names = self.find_compatible_models(controllers)

        # filter the blacklisted edges
        controllers_and_model_names_filtered = self.filter_control_and_model_names(
                PushActionEdge,
                controllers_and_model_names,
                target_iden)

        (controller, model_names) = random.choice(controllers_and_model_names_filtered)
        model_name = random.choice(model_names)

        return (controller, model_name)


    def create_drive_model(self, model_name: str) -> SystemModel:
        """ return the corresponding drive model. """

        if model_name == DRIVE_MPC_MODEL:
            return self.create_mpc_drive_model()
        elif model_name == DRIVE_MPPI_MODEL:
            return self.create_mppi_drive_model()
        else:
            raise ValueError(f"controller name unknown: {model_name}")

    def get_push_controllers(self) -> list:
        # return [self.create_mppi_push_controller_4th_order()]

        return [self.create_mppi_push_controller_4th_order(),
                self.create_mppi_push_controller_5th_order()]

    def create_push_model(self, model_name: str) -> SystemModel:
        """ create the requested push system model. """
        assert isinstance(model_name, str), f"model_name should be str and is {type(model_name)}"

        if model_name == PUSH_MPPI_MODEL:
            return self.create_mppi_push_model()
        elif model_name == PUSH_MPPI_MODEL2:
            return self.create_mppi_push_model2()
        elif model_name == PUSH_MPPI_MODEL_4TH_ORDER:
            return self.create_mppi_push_model_4th_order()
        else:
            raise ValueError(f"model name unknown: {model_name}")

    def setup_drive_controller(self, controller: DriveController, system_model: SystemModel):
        """ setup drive controller """
        assert isinstance(controller, DriveController), f"the controller should be an DriveController and is {type(controller)}"
        assert isinstance(system_model, SystemModel), f"system_model should be an SystemModel and is {type(controller)}"

        controller.setup(system_model, self.robot_obj.state, self.robot_obj.state)

    def setup_push_controller(self, controller: PushController, system_model: SystemModel, push_obj):
        """ setup push controller """

        assert isinstance(controller, PushController), f"the controller should be an PushController and is {type(controller)}"
        assert isinstance(system_model, SystemModel), f"system_model should be an SystemModel and is {type(controller)}"

        source_state = push_obj.state
        controller.setup(system_model, self.robot_obj.state, source_state, source_state)


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
    def create_mppi_push_controller_5th_order(self):
        """ create MPPI push controller. """
        return PushMppi5thOrder()

    def create_mppi_push_controller_4th_order(self):
        """ create MPPI push controller. """
        return PushMppi4thOrder()


    def create_mppi_push_model(self):
        """ create push model for MPPI pointrobot. """
        def model(x, u):

            # width square object
            H = 2

            # point a in the center of the pushed against edge
            xa = x[:,2]+torch.sin(x[:,4])*H/2
            ya = x[:,3]-torch.cos(x[:,4])*H/2

            # line parallel to the object edge being pushed against
            a_abc = torch.tan(math.pi/2-x[:,4])
            b_abc = x[:,2]+H/2*torch.sin(x[:,4])-torch.tan(math.pi/2-x[:,4])*(x[:,3]-H/2*torch.cos(x[:,4]))


            # line from center robot to center object
            a_ro = (x[:,0]-x[:,2])/(x[:,1]-x[:,3])
            b_ro = x[:,2]-(x[:,0]-x[:,2])/(x[:,1]-x[:,3])*x[:,3]

            yb = (b_ro-b_abc)/(a_abc-a_ro)
            xb = a_abc*yb+b_abc

            # st is the distance from pushing point to the centerline of the object perpendicular to the edge which is pushed against
            st=1.2*torch.sqrt((xa-xb)**2+(ya-yb)**2)

            # object rotating clockwise (positive) or anti-clockwise (negative)
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

            # width square object
            H = 2

            # point a in the center of the pushed against edge
            xa = x[:,2]+torch.sin(x[:,4])*H/2
            ya = x[:,3]-torch.cos(x[:,4])*H/2

            # line parallel to the object edge being pushed against
            a_abc = torch.tan(math.pi/2-x[:,4])
            b_abc = x[:,2]+H/2*torch.sin(x[:,4])-torch.tan(math.pi/2-x[:,4])*(x[:,3]-H/2*torch.cos(x[:,4]))


            # line from center robot to center object
            a_ro = (x[:,0]-x[:,2])/(x[:,1]-x[:,3])
            b_ro = x[:,2]-(x[:,0]-x[:,2])/(x[:,1]-x[:,3])*x[:,3]

            yb = (b_ro-b_abc)/(a_abc-a_ro)
            xb = a_abc*yb+b_abc

            # st is the distance from pushing point to the centerline of the object perpendicular to the edge which is pushed against
            st=1.2*torch.sqrt((xa-xb)**2+(ya-yb)**2)

            # object rotating clockwise (positive) or anti-clockwise (negative)
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

    def create_mppi_push_model_4th_order(self):
        """ create push model for MPPI point robot. """

        def model(x, u):
            # this model describes the robot and objects as a solid block. they move as if stuck together

            x_next = torch.zeros(x.shape, dtype=torch.float64, device=TORCH_DEVICE)

            x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT)
            x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT)
            x_next[:,2] = torch.add(x[:,2], u[:,0], alpha=0.5*DT)
            x_next[:,3] = torch.add(x[:,3], u[:,1], alpha=0.5*DT)

            return x_next

        return SystemModel(model, PUSH_MPPI_MODEL_4TH_ORDER)
    

    # TODO Make all setters and getters that are in the INIT
