"""
The hypothesis algorithm (halgorithm) takes a task and creates a hypothesis graph.
The task can be split into subtask, by adding nodes and edges a start node in a subtask
is connected to a target node. Then the halgorithm traverses the path from start to
target node by executing the edges connecting start to target node. When all edges have
succesfully been traversed, a subtask is completed. When all subtask are completed
the task is completed.
"""

from abc import abstractmethod
import random
import time
from typing import Tuple
import warnings
import numpy as np
from pyvis.network import Network

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.sphere_obstacle import SphereObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.global_planning.graph import Graph
from robot_brain.global_variables import FIG_BG_COLOR, COLORS, PROJECT_PATH, LOG_METRICS, CREATE_SERVER_DASHBOARD, SAVE_LOG_METRICS
from robot_brain.global_planning.kgraph.kgraph import KGraph

from robot_brain.global_planning.node import Node, NODE_COMPLETED, NODE_UNFEASIBLE, NODE_INITIALISED, NODE_FAILED
from robot_brain.global_planning.hgraph.object_node import ObjectNode
from robot_brain.global_planning.kgraph.change_of_state_node import ChangeOfStateNode
from robot_brain.object import Object, FREE, MOVABLE, UNKNOWN, UNMOVABLE
from robot_brain.state import State
from robot_brain.global_planning.hgraph.drive_ident_edge import DriveIdentificationEdge
from robot_brain.global_planning.edge import Edge, EDGE_INITIALISED, EDGE_COMPLETED, EDGE_EXECUTING, EDGE_FAILED
from robot_brain.global_planning.hgraph.drive_act_edge import DriveActionEdge
from robot_brain.global_planning.hgraph.push_ident_edge import PushIdentificationEdge
from robot_brain.global_planning.hgraph.push_act_edge import PushActionEdge
from robot_brain.global_planning.hgraph.action_edge import ActionEdge, EDGE_PATH_EXISTS, EDGE_PATH_IS_PLANNED, EDGE_HAS_SYSTEM_MODEL
from robot_brain.local_planning.graph_based.path_estimator import PathEstimator
from robot_brain.global_planning.hgraph.identification_edge import IdentificationEdge
from robot_brain.global_planning.hgraph.empty_edge import EmptyEdge

from robot_brain.global_planning.hgraph.hgraph import ROBOT_IDEN
from robot_brain.global_planning.hgraph.point_robot_vel_hgraph import PointRobotVelHGraph
from robot_brain.global_planning.hgraph.boxer_robot_vel_hgraph import BoxerRobotVelHGraph
from robot_brain.controller.push.push_controller import PushController
from robot_brain.controller.drive.drive_controller import DriveController
from robot_brain.system_model import SystemModel
from robot_brain.controller.controller import Controller
from robot_brain.exceptions import (
        RunnoutOfControlMethodsException,
        NoPathExistsException,
        PlanningTimeElapsedException,
        NoBestPushPositionException,
        FaultDetectedException,
        PushAnUnmovableObjectException,
        )
from logger.hlogger import HLogger

EXECUTION_LOOP = "executing"
SEARCHING_LOOP = "searching"

class HypothesisAlgorithm():
    """ Algorithm generating hypothesis that could complete a task, one subtask at a time. """

    def __init__(self, robot_obj: Object, env):

        self.env = env                  # urdf environment
        self.in_loop = SEARCHING_LOOP   # halgorithm in the search or execution loop
        self.robot_obj = robot_obj      # robot object
        self.objects = {}               # dictionary with all objects
        self.task = None                # task in form [(obst_name, target_state),..]
        self.current_subtask = None     # current subtask that the halgorithm tries to complete
        self.kgraph = None              # knowledge graph

        if robot_obj.name == "pointRobot-vel-v7":
            self.hgraph = PointRobotVelHGraph(robot_obj)
        elif robot_obj.name == "boxerRobot-vel-v7":
            self.hgraph = BoxerRobotVelHGraph(robot_obj)
        else:
            raise ValueError(f"robot name {robot_obj.name} not recognised")

        if LOG_METRICS:
            self.logger = HLogger()

        # give seed to make solutions to tasks repeatable
        random.seed(10)

        print("Hyp algorithm created")

    def setup(self, kgraph: KGraph, task: list, objects: dict):
        """
        create start and target nodes
        create task with subtasks
        create logger
        """

        self.kgraph = kgraph
        self.kgraph.print_kgraph_info()
        self.task = task
        self.objects = objects

        # let KGraph help to update object type
        for obj in self.objects.values():
            obj_type = kgraph.get_object_type(obj.name)

            if obj_type is None:
                pass
            else:
                obj.type = obj_type

        # add robot node by default
        self.hgraph.add_start_node(self.hgraph.robot_node)

        # create start and target nodes for every subtask
        for (subtask_name, (temp_obj, target)) in task.items():
            # start node
            if temp_obj != self.robot_obj:
                self.hgraph.add_start_node(ObjectNode(
                    self.hgraph.unique_node_iden(),
                    temp_obj.name,
                    temp_obj,
                    subtask_name
                    ))

            # target node
            self.hgraph.add_target_node(ObjectNode(
                self.hgraph.unique_node_iden(),
                temp_obj.name+"_target",
                Object(temp_obj.name, target, temp_obj.properties),
                subtask_name
                ))

        if CREATE_SERVER_DASHBOARD:
            self.visualise()
        if LOG_METRICS:
            self.logger.setup(task)

        print("Hyp algrotihms   is set up ")

    ##########################################
    ### execution loop #########
    ##########################################
    def respond(self) -> np.ndarray:
        """ interface into the hypothesis algorithm, eventually input for the robot is returned. """
        return np.zeros(2)

    def increment_edge(self):
        """ updates toward the next edge in the hypothesis. """

    def search_hypothesis(self):
        """ search for hypothesis using a backward search from a target node toward the start node. """


    ##########################################
    ### subtask ##############################
    ##########################################

    def update_subtask(self) -> Tuple[Node, Node]:
        """
        checks if current subtask is completed
        yes -> find new subtask and provide nodes to connect
        no -> find 2 nodes to connect
        """

    def find_nodes_to_connect_in_subtask(self) -> Tuple[Node, Node]:
        """ returns 2 nodes to connect in current subtask. """

    def find_corresponding_start_node(self, target_node):
        """ find the start node corresponding to target_node. """

    def find_source_node(self, node_iden) -> Node:
        """ find the node that points to the target_node over non-failing nodes and edges. """

    ##########################################
    ### creation of edges ####################
    ##########################################
    def create_ident_edge(self, edge: ActionEdge):
        """ creates an identification edge. """

    def create_drive_ident_edge(self, edge: DriveActionEdge):
        """ create a system identification edge. """

    def create_drive_edge(self, source_node_iden: int, target_node_iden: int):
        """ returns create drive edge and adds created model node to hgraph. """

    def create_drive_to_best_push_position_edge(self, edge: PushActionEdge):
        """ add edges/nodes to drive toward the best push position. """

    def create_push_ident_edge(self, edge: PushActionEdge):
        """ create a system identification edge for pushing. """

    def create_push_edge(self, source_node_iden: int, target_node_iden: int):
        """ returns create push edge and adds created model node to hgraph. """

    def create_remove_object_edge(self, add_node_list: list, edge: ActionEdge):
        """ add edges/nodes to remove a obj. """


    ##########################################
    ### path estimation and planning #########
    ##########################################
    def estimate_path(self, edge):
        """ estimate path existance for edge. """

        self.go_to_loop(SEARCHING_LOOP)

        assert isinstance(edge, ActionEdge), f"edge type must be ActionEdge and type is {type(edge)}"

        # path estimation
        if isinstance(edge, DriveActionEdge):
            edge.path_estimator = self.create_drive_path_estimator(self.objects)

        elif isinstance(edge, PushActionEdge):
            edge.path_estimator = self.create_push_path_estimator(self.get_node(edge.to).obj, self.objects)

        try:
            edge.path_estimator.search_path(
                self.get_node(edge.source).obj.state, self.get_node(edge.to).obj.state)
            edge.set_path_exist_status()

        except NoPathExistsException as exc:
            self.handle_no_path_exists_exception(exc, edge)

        if CREATE_SERVER_DASHBOARD:
            edge.path_estimator.visualise()


    def search_path(self, edge):
        """ search for a path from start to target for an edge. """
        # TODO: find a fix for what would happen if the start configuration of the robot is in obstacle space

        assert isinstance(edge, ActionEdge), f"edge type must be ActionEdge and type is {type(edge)}"

        if edge.motion_planner is None:

            # motion planning
            if isinstance(edge, DriveActionEdge):
                edge.motion_planner = self.create_drive_motion_planner(self.objects, edge.path_estimator)

            elif isinstance(edge, PushActionEdge):
                # edge.path_estimator.visualise(save=False)

                edge.motion_planner = self.create_push_motion_planner(
                        objects=self.objects,
                        push_obj=self.get_node(edge.source).obj,
                        path_estimator=edge.path_estimator)

        if isinstance(edge, DriveActionEdge):
            current_state = self.robot.state
        elif isinstance(edge, PushActionEdge):
            current_state = self.get_node(edge.source).obj.state

        try:
            error_triggered = False

            print(f'motion planner for obj {self.get_node(edge.source).obj.name} from {current_state.get_2d_pose()}  to {self.get_node(edge.to).obj.state.get_2d_pose()}')
            (edge.path, add_node_list) = edge.motion_planner.search_path(current_state, self.get_node(edge.to).obj.state)

        except PlanningTimeElapsedException as exc:
            error_triggered = True
            add_node_list = []

            edge.motion_planner.visualise(save=False)
            self.handle_planning_time_elapsed_exception(exc, edge)

        if error_triggered:
            return self.search_hypothesis()

        if CREATE_SERVER_DASHBOARD:
            self.visualise()
            edge.motion_planner.visualise()

        # take care of blocking object
        if len(add_node_list) > 0:
            self.create_remove_object_edge(add_node_list, edge)
            return

        edge.set_path_is_planned_status()

        # for pushing, goto best push position
        if isinstance(edge, PushActionEdge):
            self.create_drive_to_best_push_position_edge(edge)

        if CREATE_SERVER_DASHBOARD:
            self.visualise()
            edge.motion_planner.visualise()

    ##########################################
    ### handling exceptions ##################
    ##########################################

    def handle_running_out_of_control_methods_exception(self, exc: RunnoutOfControlMethodsException, target_node_iden: int):
        """ handle a RunnoutOfControlMethodsException. """

    def handle_no_path_exists_exception(self, exc: NoPathExistsException, edge: Edge):
        """ handle a NoPathExistsException. """

    def handle_planning_time_elapsed_exception(self, exc: PlanningTimeElapsedException, edge: Edge):
        """ handle a PlanningTimeElapsedException. """

    def handle_no_push_position_found_exception(self, exc: NoBestPushPositionException):
        """ handle a NoBestPushPositionException. """

    def handle_fault_detected_exception(self, exc: FaultDetectedException, edge: Edge):
        """ handle a FaultDetectedException. """

    def handle_push_an_unmovable_object_exception(self, exc: PushAnUnmovableObjectException, edge: PushActionEdge):
        """ handle a PushAnUnmovableObjectException. """

    def update_object_type_to_unmovable(self, obj_name: str):
        """ TODO: this should update the KGRAPH and other places such as hgraph

        update all nodes in hgraph, and update kgraph
        that have that store obj_name. """



    ##########################################
    ### checking / validating  ###############
    ##########################################
    def hypothesis_completed(self) -> bool:
        """ returns true if the hypothesis is completed, otherwise false. """

    def is_current_subtask_connected(self) -> bool:
        """ check if the current subtask has a path from
        source node -> target node via non-failed edges. """

    def check_for_loops(self) -> bool:
        """ returns true if the hgraph contains no loops. """
        # TODO: implement this function

    # TODO: WHihc validation functions should the also be?

    ##########################################
    ### timing ###############################
    ##########################################
    def go_to_loop(self, loop: str):
        """ go to the searching/execution loop. """

    def stop_timing(self):
        """ stop timing searching/executing. """


    ##########################################
    ### logging ##############################
    ##########################################
    def end_completed_task(self, success_ratio):
        """ finalise logs when a task completed. """

    def end_failed_task(self):
        """ finalise logs when a task failed to complete. """

    def reset_current_pointers(self):
        """ resets the current node, edge """
        #TODO: how to split this hypothesis is something that the halgorithms keeps

    ##########################################
    ### setters and getters ##################
    ##########################################
    @property
    def hypothesis(self):
        return self._hypothesis

    @hypothesis.setter
    def hypothesis(self, hyp):
        assert isinstance(hyp, list), f"hypothesis should be a list and is {type(hyp)}"
        assert all(isinstance(edge, Edge) for edge in hyp), "hypothesis should contain only edges"
        self._hypothesis = hyp

    @property
    def objects(self):
        return self._objects

    @objects.setter
    def objects(self, objects):
        assert isinstance(objects, dict),\
                f"objects should be a dictionary and is {type(objects)}"
        self._objects = objects

    @property
    def robot_order(self):
        return self._robot_order

    @robot_order.setter
    def robot_order(self, val):
        assert isinstance(val, int),\
                f"robot_order's type should be an int and is {type(val)}"
        assert val > 0, f"robot order should be higher than 0 and is {val}"
        self._robot_order = val

    def visualise(self, save=True):
        """ visualise the HGraph. """
        self.hgraph.visualise(save=save)
