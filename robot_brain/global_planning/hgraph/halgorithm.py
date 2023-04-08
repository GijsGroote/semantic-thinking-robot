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
from robot_brain.global_planning.object_node import ObjectNode
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
from robot_brain.global_planning.hgraph.hgraph import HGraph
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
        MovableObjectDetectedException,
        )
from logger.hlogger import HLogger

EXECUTION_LOOP = "executing"
SEARCHING_LOOP = "searching"

"""

    SOME IMPORTANT NOTES:

    start/target nodes have a subtask
    regular nodes do not have a subtask
    edges have a subtask



"""

class HypothesisAlgorithm():
    """ Algorithm generating hypothesis that could complete a task, one subtask at a time. """

    def __init__(self, robot_obj: Object, env):

        self.env = env                  # urdf environment
        self.in_loop = SEARCHING_LOOP   # halgorithm in the search or execution loop
        self.robot_obj = robot_obj      # robot object
        self.task = None                # task in form [(obst_name, target_state),..]
        self.objects = {}               # dictionary with all objects
        self.hypothesis = []            # list of edges, current hypothesis to complete self.current_subtask
        self.edge_pointer = 0           # point toward the self.current_edge in self.hypothesis
        self.current_edge = None
        self.current_node = None
        self.current_subtask = None     # current subtask that the halgorithm tries to complete
        self.kgraph = None              # knowledge graph

        if robot_obj.name == "pointRobot-vel-v7":
            self.hgraph = PointRobotVelHGraph(robot_obj)
        # elif robot_obj.name == "boxerRobot-vel-v7":
        #     self.hgraph = BoxerRobotVelHGraph(robot_obj)
        else:
            raise ValueError(f"robot name {robot_obj.name} not recognised")

        if LOG_METRICS:
            self.logger = HLogger()

        # give seed to make solutions to tasks repeatable
        random.seed(10)

    def setup(self, kgraph: KGraph, task: list, objects: dict):
        """
        mainly, create start and target nodes:
            driving subtask receives 2 nodes (start, target for robot)
            pushing subtask receives 3 nodes (robot, start obj, target obj)

        marginally, create logger, objects list, among other.
        """

        self.kgraph = kgraph
        self.kgraph.print_kgraph_info()
        self.task = task
        self.objects = objects

        # let KGraph help to update objects type
        for obj in self.objects.values():
            obj_type = kgraph.get_object_type(obj.name)

            if obj_type is not None:
                obj.type = obj_type

        # create start and target nodes for every subtask
        for (subtask_name, (temp_obj, target)) in task.items():

            # robot start node
            self.hgraph.add_start_node(ObjectNode(
                self.hgraph.unique_node_iden(),
                self.robot_obj.name,
                self.robot_obj,
                subtask_name
                ))

            # start object node
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

    ##########################################
    ### execution loop #########
    ##########################################
    def respond(self) -> np.ndarray:
        """ interface into the hypothesis algorithm, eventually input for the robot is returned. """

        if self.in_loop == EXECUTION_LOOP:

            if self.current_edge.completed():
                self.increment_edge()

            try:

                return self.current_edge.respond()

            except MovableObjectDetectedException as exc:
                # TODO: update objects with type is movable
                pass

            except PushAnUnmovableObjectException as exc:
                self.handle_push_an_unmovable_object_exception(exc, self.current_edge)

            except FaultDetectedException as exc:
                # TODO: fault detection could be done here if fault -> raise FaultDetectedException('tsting this ')
                pass

            self.search_hypothesis()
            return self.respond()

        elif self.in_loop == SEARCHING_LOOP:
            self.search_hypothesis()
            return self.respond()

        else:
            raise ValueError(f"Halgorithm should be in search or execution loop an is in: {self.in_loop}")


    def search_hypothesis(self):
        """ search for hypothesis using a backward search from a target node toward the start node. """

        self.go_to_loop(SEARCHING_LOOP)

        if not self.is_current_subtask_connected():

            # find nodes in new subtask or current unfinished subtask
            (start_node, target_node) = self.update_subtask()

            # connect the unconnected start to target node
            self.connect_nodes_with_action_edge(start_node, target_node)

            # update current node and current edge
            self.update_current_node_and_edge()

        # prepare the edge current_edge for execution
        self.make_ready_for_execution()

        # add ghost pose
        self.add_target_ghost()

        self.current_edge.set_executing_status()
        self.go_to_loop(EXECUTION_LOOP)

        if CREATE_SERVER_DASHBOARD:
            self.visualise()

    def connect_nodes_with_action_edge(self, start_node: ObjectNode, target_node: ObjectNode):
        """ create an action edge between the start and target node. """
        assert not self.hgraph.is_reachable(start_node, target_node), "start nodes is already connected to target node"
        assert start_node.obj.name == target_node.obj.name,\
                f"object: {start_node.name} in start_node should be equal to object: {target_node.name} in target_node"

        # driving action
        if target_node.obj.properties == self.robot_obj.properties:
            self.create_drive_edge(start_node.iden, target_node.iden)

        # pushing action
        elif target_node.obj.properties != self.robot_obj.properties:
            self.create_push_edge(start_node.iden, target_node.iden)

        else:
            raise ValueError(f"Cannot connect start node {start_node.iden} to target node {target_node.iden}")

    def make_ready_for_execution(self):
        """ recursively make the first edge in hypothesis ready for execution. """
        assert isinstance(self.current_edge, Edge), f"current_edge must be an Edge and is: {type(self.current_edge)}"

        # check if the first edge is planned.
        if not self.current_edge.ready_for_execution():

            # make the action edge ready
            if isinstance(self.current_edge, ActionEdge):
                if self.current_edge.status == EDGE_INITIALISED:
                    self.estimate_path(self.current_edge)

                elif self.current_edge.status == EDGE_PATH_EXISTS:
                    self.create_ident_edge(self.current_edge)

                elif self.current_edge.status == EDGE_HAS_SYSTEM_MODEL:
                    self.search_path(self.current_edge)

                elif self.current_edge.status == EDGE_EXECUTING:
                    return

                elif self.current_edge.status in [EDGE_PATH_IS_PLANNED, EDGE_COMPLETED, EDGE_FAILED]:
                    ValueError("path is {self.current_edge.iden}, and that in this section should be impossible.")

                else:
                    ValueError(f"unknown status for edge {self.current_edge.iden} and status {self.current_edge.status}")

                return self.make_ready_for_execution()

            # make the identification edge ready
            elif isinstance(self.current_edge, IdentificationEdge):

                # NOTE: because of hard-coded system models making the identification edge
                # ready for execution (the code below) is not doing anything really. It serves
                # as a start when system identification is added to the project. Gijs Groote 8-4-2023.

                if self.current_edge.status == EDGE_INITIALISED:
                    pass

                elif self.current_edge.status == EDGE_EXECUTING:
                    print(f"current identification edge {self.current_edge.iden} is already execution")
                    return

                elif self.current_edge.status == EDGE_FAILED:
                    print(f'current ident edge status = {self.current_edge.status}')

                else:
                    ValueError(f"unknown status for edge {self.current_edge.iden} and status {self.current_edge.status}")

                return self.make_ready_for_execution()



    ##########################################
    ### find which nodes to connect ##########
    ##########################################

    def is_current_subtask_connected(self) -> bool:
        """ check if the current subtask has a path from
        starting node -> target node via non-failed edges
        and in the same subtask. """
        if self.current_subtask is None:
            self.update_subtask()

        return self.hgraph.is_reachable(
                self.current_subtask["start_node"],
                self.current_subtask["target_node"])

    def update_subtask(self) -> Tuple[Node, Node]:
        """
        checks if current subtask is completed
        yes -> find new subtask and provide nodes to connect
        no -> find 2 nodes to connect
        """
        if self.current_subtask is None or self.current_subtask["target_node"].status != NODE_INITIALISED:

            # check if all tasks are completed
            # if all subtask are completed, conclude the task is completed, otherwise search new hypothesis
            if all(target_node.status != NODE_INITIALISED for target_node in self.hgraph.target_nodes.values()):
                self.finish_task()

            # reset pointers
            self.reset_current_pointers()

            # find a new subtask
            unfinished_target_nodes = [target_node for
                    target_node in self.hgraph.target_nodes.values()
                    if target_node.status == NODE_INITIALISED]

            object_target_node = None
            robot_target_node = None

            # first push then drive subtasks
            for unfinished_target_node in unfinished_target_nodes:
                if unfinished_target_node.obj == self.robot_obj:
                    robot_target_node = unfinished_target_node
                else:
                    object_target_node = unfinished_target_node

            if object_target_node is None:
                if robot_target_node is None:
                    self.end_failed_task()
                    raise StopIteration("No more subtasks, No Solution Found!")

                subtask_start_node = self.hgraph.get_start_node_from_target_node(robot_target_node)
                subtask_target_node = robot_target_node
            else:
                subtask_start_node = self.hgraph.get_start_node_from_target_node(object_target_node)
                subtask_target_node = object_target_node


            self.current_subtask = {
                    "start_node": subtask_start_node,
                    "target_node": subtask_target_node,
                    "name": subtask_target_node.subtask_name,
                    "now_timing": None,
                    "start_time": None,
                    "execute_time": 0.0,
                    "search_time": 0.0,

                    }

            print(f'makign subtask heyhey {self.current_subtask}')

            print(f'heyhey {subtask_start_node.iden} ha {subtask_target_node.iden}')
            return (subtask_start_node, subtask_target_node)

        else:
            # focus on current subtask
            return self.get_nodes_to_connect_in_subtask()

    def finish_task(self):
        """ task is finished, complete logs and raise StopIteration. """
        n_completed_subtasks = 0
        n_failed_subtasks = 0
        n_unfeasible_subtasks = 0

        for target_node in self.hgraph.target_nodes.values():
            if target_node.status == NODE_COMPLETED:
                n_completed_subtasks += 1
            elif target_node.status == NODE_FAILED:
                n_failed_subtasks += 1
            elif target_node.status == NODE_UNFEASIBLE:
                n_unfeasible_subtasks += 1
            else:
                raise ValueError(f"target node status should be {NODE_COMPLETED}, {NODE_FAILED} or {NODE_UNFEASIBLE}"\
                        f" and is {target_node.status} for node {target_node.iden}")

        if n_failed_subtasks+n_completed_subtasks == 0:
            task_success_ratio = None
        else:
            task_success_ratio = n_completed_subtasks/len(self.task)

        self.end_completed_task(task_success_ratio)

        raise StopIteration(f"The task is completed with a success/fail ration of {task_success_ratio}!")

    def get_nodes_to_connect_in_subtask(self) -> Tuple[Node, Node]:
        """ returns 2 nodes to connect in current subtask. """
        assert self.current_subtask is not None, "subtask is None"
        target_node = self.hgraph.get_connected_source_node(self.current_subtask["target_node"], self.current_subtask["name"])

        assert target_node.status == NODE_INITIALISED,\
                f"target node status must be {NODE_INITIALISED} and is {target_node.status} for node {target_node.iden}"
        start_node = self.hgraph.get_connected_target_node(self.current_subtask["start_node"], self.current_subtask["name"])

        assert start_node.status == NODE_INITIALISED,\
                f"start node status must be {NODE_INITIALISED} and is {start_node.status} for node {target_node.iden}"
        assert start_node.iden != target_node.iden, f"start node is target node, which are: {start_node.iden}"

        return (start_node, target_node)


    ##########################################
    ### creation of edges ####################
    ##########################################
    def create_ident_edge(self, edge: ActionEdge):
        """ creates an identification edge. """
        if isinstance(edge, DriveActionEdge):
            self.create_drive_ident_edge(edge)
        elif isinstance(edge, PushActionEdge):
            self.create_push_ident_edge(edge)
        else:
            raise ValueError

    def create_drive_ident_edge(self, edge: DriveActionEdge):
        """ create a system identification edge. """
        assert isinstance(edge, DriveActionEdge)

        model_node = ObjectNode(
                iden=self.hgraph.unique_node_iden(),
                name=self.robot_obj.name+"_model",
                obj=self.robot_obj,
                subtask_name=self.hgraph.get_node(edge.to).subtask_name)

        self.hgraph.add_node(model_node)

        drive_ident_edge = DriveIdentificationEdge(
                iden=self.hgraph.unique_edge_iden(),
                source=edge.source,
                to=model_node.iden,
                verb="identify",
                controller="controller",
                model_for_edge_iden=edge.iden,
                subtask_name=self.hgraph.get_node(edge.to).subtask_name)

        edge.source = model_node.iden
        drive_ident_edge.system_model = self.hgraph.create_drive_model(edge.model_name)
        self.hgraph.add_edge(drive_ident_edge)
        self.add_edge_to_hypothesis(drive_ident_edge)

    def create_drive_edge(self, source_node_iden: int, target_node_iden: int):
        """ create drive edge and adds created model node to hgraph. """

        # TODO: the knowledge graph should come into play here
        knowledge_graph = False

        if knowledge_graph:
            print("knowledge graph has a proposition")
            # TODO: the knowledge graph should come into play here
        else:

            exception_triggered = False
            try:
                (controller, model_name) = self.hgraph.create_drive_controller(target_node_iden)

            except RunnoutOfControlMethodsException as exc:
                exception_triggered = True
                self.handle_running_out_of_control_methods_exception(exc, target_node_iden)

            if exception_triggered:
                return self.search_hypothesis()

            edge = DriveActionEdge(iden=self.hgraph.unique_edge_iden(),
                    source=source_node_iden,
                    to=target_node_iden,
                    robot_obj=self.robot_obj,
                    verb="driving",
                    controller=controller,
                    model_name=model_name,
                    subtask_name=self.hgraph.get_node(target_node_iden).subtask_name)

            self.hgraph.add_edge(edge)
            self.add_edge_to_hypothesis(edge)


    def create_drive_to_best_push_position_edge(self, edge: PushActionEdge):
        """ add edges/nodes to drive toward the best push position. """
        # TODO: Sunday

    def create_push_ident_edge(self, edge: PushActionEdge):
        """ create a system identification edge for pushing. """
        # TODO: Sunday

    def create_push_edge(self, source_node_iden: int, target_node_iden: int):
        """ returns create push edge and adds created model node to hgraph. """
        # TODO: Sunday

    def create_remove_object_edge(self, add_node_list: list, edge: ActionEdge):
        """ add edges/nodes to remove a obj. """
        # TODO: Sunday


    ##########################################
    ### path estimation and planning #########
    ##########################################
    def estimate_path(self, edge):
        """ estimate path existance for edge. """

        assert isinstance(edge, ActionEdge), f"edge type must be ActionEdge and type is {type(edge)}"

        # path estimation
        if isinstance(edge, DriveActionEdge):
            edge.path_estimator = self.hgraph.create_drive_path_estimator(self.objects)

        elif isinstance(edge, PushActionEdge):
            edge.path_estimator = self.hgraph.create_push_path_estimator(self.hgraph.get_node(edge.to).obj, self.objects)

        try:
            edge.path_estimator.search_path(
                self.hgraph.get_node(edge.source).obj.state, self.hgraph.get_node(edge.to).obj.state)
            edge.set_path_exist_status()

        except NoPathExistsException as exc:
            self.handle_no_path_exists_exception(exc, edge)

        if CREATE_SERVER_DASHBOARD:
            edge.path_estimator.visualise()


    def search_path(self, edge):
        """ search for a path from start to target for an edge. """
        assert isinstance(edge, ActionEdge), f"edge type must be ActionEdge and type is {type(edge)}"

        if edge.motion_planner is None:

            # motion planning
            if isinstance(edge, DriveActionEdge):
                edge.motion_planner = self.hgraph.create_drive_motion_planner(self.objects, edge.path_estimator)

            # manipulation planning
            elif isinstance(edge, PushActionEdge):

                edge.motion_planner = self.hgraph.create_push_motion_planner(
                        objects=self.objects,
                        push_obj=self.hgraph.get_node(edge.source).obj,
                        path_estimator=edge.path_estimator)

        if isinstance(edge, DriveActionEdge):
            current_state = self.hgraph.get_node(edge.source).obj.state
        elif isinstance(edge, PushActionEdge):
            current_state = self.hgraph.get_node(edge.source).obj.state

        try:
            error_triggered = False
            (edge.path, add_node_list) = edge.motion_planner.search_path(current_state, self.hgraph.get_node(edge.to).obj.state)

        except PlanningTimeElapsedException as exc:
            error_triggered = True
            add_node_list = []

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
        assert isinstance(exc, RunnoutOfControlMethodsException)
        assert isinstance(target_node_iden, int)

        self.hgraph.get_node(target_node_iden).status = NODE_FAILED

        # make outgoing edges unfeasible and remove from hypothesis
        for temp_edge in self.hgraph.get_outgoing_edges(target_node_iden):
            self.hgraph.fail_edge(temp_edge)
            self.hgraph.add_to_blocklist(temp_edge)

            if temp_edge in self.hypothesis:
                self.hypothesis.remove(temp_edge)

        self.log_exception(exc, edge)

    def handle_no_path_exists_exception(self, exc: NoPathExistsException, edge: ActionEdge):
        """ handle a NoPathExistsException. """
        assert isinstance(exc, NoPathExistsException)
        assert isinstance(edge, ActionEdge)

        self.hgraph.get_node(edge.to).status = NODE_UNFEASIBLE
        for outgoing_edge in self.hgraph.get_outgoing_edges(edge.to):
            self.hgraph.fail_edge(outgoing_edge)

        self.hgraph.fail_edge(edge)

        self.log_exception(exc, edge)


    def handle_planning_time_elapsed_exception(self, exc: PlanningTimeElapsedException, edge: ActionEdge):
        """ handle a PlanningTimeElapsedException. """
        assert isinstance(exc, PlanningTimeElapsedException)
        assert isinstance(edge, ActionEdge)

        self.hgraph.add_to_blocklist(edge)
        self.hgraph.fail_edge(edge)
        self.current_edge = None

        # remove all edges up to the failed edge from the current hypothesis.
        self.hypothesis = self.hypothesis[self.hypothesis.index(edge)+1:]
        self.edge_pointer = 0

        self.log_exception(exc, edge)

    def handle_no_push_position_found_exception(self, exc: NoBestPushPositionException):
        """ handle a NoBestPushPositionException. """
        # TODO: Sunday

    def handle_fault_detected_exception(self, exc: FaultDetectedException, edge: Edge):
        """ handle a FaultDetectedException. """
        # TODO: Sunday

    def handle_push_an_unmovable_object_exception(self, exc: PushAnUnmovableObjectException, edge: PushActionEdge):
        """ handle a PushAnUnmovableObjectException. """
        # TODO: Sunday

    def log_exception(self, exc: Exception, edge: Edge):
        """ add to logger and visualise. """
        assert isinstance(exc, Exception)

        if LOG_METRICS:
            self.logger.add_failed_hypothesis(self.hypothesis, self.current_subtask, str(exc))

        if CREATE_SERVER_DASHBOARD:
            self.visualise()
            edge.path_estimator.visualise()
            edge.motion_planner.visualise()

    def update_object_type_to_unmovable(self, obj_name: str):
        """ TODO: this should update the KGRAPH and other places such as hgraph

        update all nodes in hgraph, and update kgraph
        that have that store obj_name. """
        # TODO: Saturday


    # TODO: Saturday: make a function that finds if an object is movable and update objects and KGraph


    ##########################################
    ### checking / validating  ###############
    ##########################################
    def hypothesis_completed(self) -> bool:
        """ returns true if the hypothesis is completed, otherwise false. """
        return self.edge_pointer >= len(self.hypothesis)-1


    ##########################################
    ### timing ###############################
    ##########################################
    def go_to_loop(self, loop: str):
        """ go to the searching/execution loop. """

        if loop in [EXECUTION_LOOP, SEARCHING_LOOP]:

            self.in_loop = loop

            if LOG_METRICS and self.current_subtask is not None:
                self.stop_timing()
                self.current_subtask["start_time"] = time.time()
                self.current_subtask["now_timing"] = loop

        else:
            raise ValueError(f"unknown loop: {loop}")

    def stop_timing(self):
        """ stop timing searching/executing. """
        """ stop timing searching/executing. """
        if LOG_METRICS:
            if self.current_subtask["now_timing"] is None:
                pass
            elif self.current_subtask["now_timing"] == EXECUTION_LOOP:
                self.current_subtask["execute_time"] += time.time() - self.current_subtask["start_time"]

            elif self.current_subtask["now_timing"] == SEARCHING_LOOP:
                self.current_subtask["search_time"] += time.time() - self.current_subtask["start_time"]

            else:
                raise ValueError(f"now_timing has an unexpected value: {self.current_subtask['now_timing']}")

            self.current_subtask["now_timing"] = None
            self.current_subtask["start_time"] = None # you could leave this out



    ##########################################
    ### logging ##############################
    ##########################################
    def end_completed_task(self, success_ratio):
        """ finalise logs when a task completed. """
        # TODO: saturday

    def end_failed_task(self):
        """ finalise logs when a task failed to complete. """
        # TODO: saturday


    ##########################################
    ### maintaining the hypothesis ###########
    ##########################################
    def add_edge_to_hypothesis(self, edge:Edge):
        """ adds edge to hypothesis. """
        self.hypothesis.insert(self.edge_pointer, edge)

        # update current_node and current_edge
        self.update_current_node_and_edge()


    def increment_edge(self):
        """ updates toward the next edge in the hypothesis. """

        # identification edge completed -> update a system model
        if isinstance(self.current_edge, IdentificationEdge):
            self.hgraph.update_system_model(self.current_edge)

        # update KGraph
        elif isinstance(self.current_edge, ActionEdge):
            self.kgraph.add_edge_review(self.hgraph.get_node(self.current_edge.source).obj, self.current_edge)

        # complete current edge
        self.current_edge.set_completed_status()

        if self.hypothesis_completed():
            # self.stop_timing()
            if LOG_METRICS:
                self.logger.add_succesfull_hypothesis(self.hypothesis, self.current_subtask)
            self.hgraph.get_node(self.current_edge.to).status = NODE_COMPLETED
            self.reset_current_pointers()
            return self.search_hypothesis()

        self.hgraph.get_node(self.current_edge.to).status = NODE_COMPLETED
        self.edge_pointer += 1

        # search_hypothesis checks the validity of the next edge
        self.search_hypothesis()

    def reset_current_pointers(self):
        """ resets the current node, edge """
        self.current_subtask = None
        self.current_node = None
        self.current_edge = None
        self.hypothesis = []
        self.edge_pointer = 0


    def update_current_node_and_edge(self):
        """ update the current node and current edge. """
        self.edge_pointer = self.edge_pointer


    ##########################################
    ### setters and getters ##################
    ##########################################
    @property
    def env(self):
        return self._env

    @env.setter
    def env(self, val):
        self._env = val

    @property
    def in_loop(self):
        return self._in_loop

    @in_loop.setter
    def in_loop(self, val):
        self._in_loop = val

    @property
    def robot_obj(self):
        return self._robot_obj

    @robot_obj.setter
    def robot_obj(self, val):
        assert isinstance(val, Object)
        self._robot_obj = val

    @property
    def task(self):
        return self._task

    @task.setter
    def task(self, val):
        self._task = val

    @property
    def objects(self):
        return self._objects

    @objects.setter
    def objects(self, objects):
        assert isinstance(objects, dict),\
                f"objects should be a dictionary and is {type(objects)}"
        self._objects = objects

    @property
    def hypothesis(self):
        return self._hypothesis

    @hypothesis.setter
    def hypothesis(self, hyp):
        assert isinstance(hyp, list), f"hypothesis should be a list and is {type(hyp)}"
        assert all(isinstance(edge, Edge) for edge in hyp), "hypothesis should contain only edges"
        self._hypothesis = hyp

    @property
    def edge_pointer(self):
        return self._edge_pointer

    @edge_pointer.setter
    def edge_pointer(self, val):
        assert isinstance(val, int), f"edge_pointer should be an int and is {type(val)}"
        assert val >= 0
        if len(self.hypothesis) > 0:
            self.current_edge = self.hypothesis[val]
            self.current_node = self.hgraph.get_node(self.current_edge.source)

        self._edge_pointer = val

    @property
    def current_edge(self) -> Edge:
        return self._current_edge

    @current_edge.setter
    def current_edge(self, edge: Edge):
        assert isinstance(edge, (Edge, type(None))), f"edge should be an Edge or None and is {type(edge)}"
        if isinstance(edge, Edge):
            self.hgraph.c_edge = edge
        self._current_edge = edge

    @property
    def current_node(self):
        return self._current_node

    @current_node.setter
    def current_node(self, node) -> ObjectNode:
        assert isinstance(node, (Node, type(None))), f"node should be a Node or None and is {type(node)}"
        if isinstance(node, Node):
            assert node.iden in self.hgraph.nodes, "node should be in self.nodes"
            self.hgraph.c_node = node
        self._current_node = node

    @property
    def current_subtask(self):
        return self._current_subtask

    @current_subtask.setter
    def current_subtask(self, val):
        assert isinstance(val, (dict, type(None))), f"current_subtask should be a dictionary and is {type(val)}"

        self._current_subtask = val

    @property
    def hgraph(self):
        return self._hgraph

    @hgraph.setter
    def hgraph(self, val):
        assert isinstance(val, HGraph), f"hgraph should be a HGraph and is {type(val)}"
        self._hgraph = val

    @property
    def kgraph(self):
        return self._kgraph

    @kgraph.setter
    def kgraph(self, val):
        assert isinstance(val, (KGraph, type(None))), f"kgraph should be a KGraph and is {type(val)}"
        self._kgraph = val

    @property
    def logger(self):
        return self._logger

    @logger.setter
    def logger(self, val):
        assert isinstance(val, HLogger), f"logger should be a HLogger and is {type(val)}"
        self._logger = val

    ##########################################
    ### visualise functionality ##############
    ##########################################
    def add_target_ghost(self):
        """ add target ghost and remove old target ghost. """

        if isinstance(self.current_edge, PushActionEdge):
            self.env.add_target_ghost(
                self.hgraph.get_node(self.current_edge.to).obj.properties.name(),
                self.hgraph.get_node(self.current_edge.to).obj.state.get_2d_pose()
                )

        elif isinstance(self.current_edge, DriveActionEdge):
            self.env.add_robot_target_ghost(
                self.robot_obj.name,
                self.hgraph.get_node(self.current_edge.to).obj.state.get_2d_pose()
                )

    def visualise(self, save=True):
        """ visualise the HGraph. """
        self.hgraph.visualise(save=save)
