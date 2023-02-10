from abc import abstractmethod
import random
import time
from typing import Tuple
import numpy as np
from pyvis.network import Network
import warnings

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.sphere_obstacle import SphereObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.global_planning.graph import Graph
from robot_brain.global_variables import FIG_BG_COLOR, COLORS, PROJECT_PATH, LOG_METRICS, CREATE_SERVER_DASHBOARD, SAVE_LOG_METRICS
# from robot_brain.global_planning.kgraph.kgraph import KGraph

from robot_brain.global_planning.node import Node, NODE_COMPLETED, NODE_UNFEASIBLE, NODE_INITIALISED, NODE_FAILED
from robot_brain.global_planning.obstacle_node import ObstacleNode
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.obstacle import Obstacle
from robot_brain.state import State
from robot_brain.global_planning.drive_ident_edge import DriveIdentificationEdge
from robot_brain.global_planning.edge import Edge, EDGE_INITIALISED, EDGE_COMPLETED, EDGE_EXECUTING, EDGE_FAILED
from robot_brain.global_planning.drive_act_edge import DriveActionEdge
from robot_brain.global_planning.push_ident_edge import PushIdentificationEdge
from robot_brain.global_planning.push_act_edge import PushActionEdge
from robot_brain.global_planning.action_edge import ActionEdge, EDGE_PATH_EXISTS, EDGE_PATH_IS_PLANNED, EDGE_HAS_SYSTEM_MODEL
from robot_brain.global_planning.hgraph.local_planning.graph_based.path_estimator import PathEstimator
from robot_brain.global_planning.identification_edge import IdentificationEdge
from robot_brain.global_planning.empty_edge import EmptyEdge
from robot_brain.controller.push.push_controller import PushController
from robot_brain.controller.drive.drive_controller import DriveController
from robot_brain.system_model import SystemModel
from robot_brain.controller.controller import Controller
from robot_brain.exceptions import (
        RunnoutOfControlMethodsException,
        NoPathExistsException,
        PlanningTimeElapsedException,
        FaultDetectedException
        )
from logger.hlogger import HLogger

EXECUTION_LOOP = "executing"
SEARCHING_LOOP = "searching"
ROBOT_IDEN = 0

# TODO: make quite a lot of methods private
class HGraph(Graph):
    """
    Hypothesis graph.
    """
    def __init__(self, env):
        Graph.__init__(self)

        self.env = env                 # urdf environment
        self.in_loop = SEARCHING_LOOP  # hgraph can be in the execution or search loop
        self.task = None               # task in form [(obst_name, target_state),..]
        self.robot_node = None         # node containing the robot itself
        self.obstacles = {}            # dictionary with all obstacles
        self.start_nodes = []          # starting nodes one for every subtask and one for the robot
        self.target_nodes = []         # target nodes one for every subtask
        self.start_to_target_iden = [] # identifier mapping from start to target node and vise versa
        self.blacklist = {}            # blacklist containing banned edges
        self.current_subtask = None    # current subtask that the HGraph tries to complete

        # self.current_edge is the current edge being executed or first in line to be executed.
        # self.current_node is the source node of the current edge being executed for plotting purposes

        self.reset_current_pointers()
        if LOG_METRICS:
            self.logger = HLogger()

    def setup(self, task, obstacles):
        """ create start and target nodes and adds them to the hgraph. """

        self.task = task
        self.obstacles = obstacles

        #  add robot as start_state
        self.robot_node = ObstacleNode(ROBOT_IDEN, self.robot.name, self.robot)
        self.current_node = self.robot_node
        self.add_start_node(self.robot_node)

        for (subtask_name, (obst_temp, target)) in task.items():
            if obst_temp == self.robot:
                iden_start_node = ROBOT_IDEN
            else:
                iden_start_node = self.unique_node_iden()
                self.add_start_node(ObstacleNode(
                    iden_start_node,
                    obst_temp.name,
                    obst_temp,
                    subtask_name
                    ))
            iden_target_node = self.unique_node_iden()
            self.add_target_node(ObstacleNode(
                iden_target_node,
                obst_temp.name+"_target",
                Obstacle(obst_temp.name, target, obst_temp.properties),
                subtask_name
                ))
            self.start_to_target_iden.append((iden_start_node, iden_target_node))


        self.update_subtask()

        if CREATE_SERVER_DASHBOARD:
            self.visualise()
        if LOG_METRICS:
            self.logger.setup(task)


    ##############################################################################################
    ################ MAIN FUNCTIONS ######################### MAIN FUNCTIONS #####################
    ##############################################################################################
    def respond(self) -> np.ndarray:
        """ interface toward the robot simulation, eventually input for the robot is returned. """

        if self.in_loop == EXECUTION_LOOP:

            # TODO: fault detection could be done here if fault -> raise FaultDetectedException('tsting this ')

            if self.current_edge.completed():

                self.increment_edge()

            return self.current_edge.respond()

        elif self.in_loop == SEARCHING_LOOP:
            self.search_hypothesis()
            return self.respond()

        else:
            raise ValueError(f"HGraph in an unknown loop: {self.in_loop}")

    def search_hypothesis(self):
        """ Search by backward induction from a target node toward the robot start node. """

        if not self.is_current_subtask_connected():

            (start_node, target_node) = self.update_subtask()
            # find nodes in new subtask or current unfinished subtask
            self.go_to_loop(SEARCHING_LOOP)
            print(f'not connected connect start node {start_node.name} and target: {target_node.name}')

            # search for unfinished target node and connect to starting node
            # TODO: hypothesis should be reachable, but also the first edge in hypothesis (or the first after the current edge) should be

            while not self.is_reachable(self.current_subtask["start_node"].iden, self.current_subtask["target_node"].iden):

                # the obstacles should be the same between an edge
                assert start_node.obstacle.name == target_node.obstacle.name,\
                f"obstacle: {start_node.name} in start_node should be equal to obstacle: {target_node.name} in target_node"

                # driving action
                if target_node.obstacle.properties == self.robot.properties:
                    print("create drive edge")
                    self.create_drive_edge(start_node.iden, target_node.iden)

                # pushing action
                # TODO make more sure that this requires a pushing action
                else:
                    print("create push edge")
                    self.create_push_edge(start_node.iden, target_node.iden)


                # find_subtask is not forced to find the same final_target_node
                (start_node, target_node) = self.find_nodes_to_connect_in_subtask()

        self.current_edge = self.hypothesis[self.edge_pointer]
        self.current_node = self.get_node(self.current_edge.source)

        # check if the first edge is planned.
        if not self.current_edge.ready_for_execution():

            print(f"above edge {self.current_edge.iden} is not yet ready it's at {self.current_edge.status}")
            print(f'is it ready: {self.current_edge.ready_for_execution()}')

            # make the action edge ready
            if isinstance(self.current_edge, ActionEdge):
                if self.current_edge.status == EDGE_INITIALISED:
                    self.estimate_path(self.current_edge)

                elif self.current_edge.status == EDGE_PATH_EXISTS:
                    self.create_ident_edge(self.current_edge)

                elif self.current_edge.status == EDGE_HAS_SYSTEM_MODEL:
                    print('start path search please')
                    self.search_path(self.current_edge)

                elif self.current_edge.status == EDGE_EXECUTING:
                    print(f"current action edge {self.current_edge.iden} is already execution")
                    return

                elif self.current_edge.status in [EDGE_PATH_IS_PLANNED, EDGE_COMPLETED, EDGE_FAILED]:
                    ValueError("path is {self.current_edge.iden}, and that in this section should be impossible.")

                else:
                    ValueError(f"unknown status for edge {self.current_edge.iden} and status {self.current_edge.status}")

            return self.search_hypothesis()

        # make the identification edge ready
        if not self.current_edge.ready_for_execution():
            if isinstance(self.current_edge, IdentificationEdge):

                if self.current_edge.status == EDGE_INITIALISED:
                    print(f'current ident edge status = {self.current_edge.status}')
                    # self.estimate_path(self.current_edge)

                elif self.current_edge.status == EDGE_EXECUTING:
                    print(f"current identification edge {self.current_edge.iden} is already execution")
                    return

                elif self.current_edge.status == EDGE_FAILED:
                    print(f'current ident edge status = {self.current_edge.status}')
                    # self.search_path(self.current_edge)

                else:
                    ValueError(f"unknown status for edge {self.current_edge.iden} and status {self.current_edge.status}")


            # if self.current_edge.iden == 6:
            #     for temp_edge in self.hypothesis:
            #         print(f'hyp {temp_edge.iden}')
            #
            #     print(f"edge {self.current_edge.iden} is not yet ready it's at {self.current_edge.status}")
            #     self.visualise(save=False)
            #     raise ValueError
            return self.search_hypothesis()

        # add ghost pose
        if isinstance(self.current_edge, PushActionEdge):
            self.env.add_target_ghost(self.get_node(self.current_edge.to).obstacle.properties.name(),
            self.get_node(self.current_edge.to).obstacle.state.get_2d_pose())


        print('current hyp is like this')
        for temp_edge in self.hypothesis:
            print(f'hyp {temp_edge.iden}')
        self.visualise(save=False) # delete this

        self.current_edge.set_executing_status()
        self.go_to_loop(EXECUTION_LOOP)
        if CREATE_SERVER_DASHBOARD:
            self.visualise()

    def create_drive_edge(self, source_node_iden: int, target_node_iden: int):
        """ returns create drive edge and adds created model node to hgraph. """

        knowledge_graph = False

        if knowledge_graph:
            print("knowledge graph has a proposition")
            # TODO: the knowledge graph should come into play here
        else:

            try:
                (controller, model_name) = self.create_drive_controller(target_node_iden)

            except RunnoutOfControlMethodsException as exc:
                self.handle_running_out_of_control_methods_exception(exc, target_node_iden)

            edge = DriveActionEdge(iden=self.unique_edge_iden(),
                    source=source_node_iden,
                    to=target_node_iden,
                    robot_obst=self.robot,
                    verb="driving",
                    controller=controller,
                    model_name=model_name)

            self.add_edge(edge)
            self.hypothesis.insert(self.edge_pointer, edge)

    def create_push_edge(self, source_node_iden: int, target_node_iden: int):
        """ returns create push edge and adds created model node to hgraph. """

        knowledge_graph = False

        if knowledge_graph:
            print("knowledge graph has a proposition")
        else:

            print(f'create push edge between {source_node_iden} and {target_node_iden}')

            # Create a PushActionEdge
            exception_triggered = False
            try:
                (controller, model_name) = self.create_push_controller(target_node_iden)

            except RunnoutOfControlMethodsException as exc:
                exception_triggered = True
                self.handle_running_out_of_control_methods_exception(exc, target_node_iden)

            if exception_triggered:
                return self.search_hypothesis()

            start_node = self.get_node(source_node_iden)
            target_node = self.get_node(target_node_iden)

            edge = PushActionEdge(
                    iden=self.unique_edge_iden(),
                    source=source_node_iden,
                    to=target_node_iden,
                    robot_obst=self.robot,
                    push_obst=start_node.obstacle,
                    verb="pushing",
                    controller=controller,
                    model_name=model_name)

            self.add_edge(edge)
            self.hypothesis.insert(self.edge_pointer, edge)

    def estimate_path(self, edge):
        """ Estimate path existance for from start to target for an edge. """

        self.go_to_loop(SEARCHING_LOOP)

        assert isinstance(edge, ActionEdge), f"edge type must be ActionEdge and type is {type(edge)}"

        # path estimation
        if isinstance(edge, DriveActionEdge):
            edge.path_estimator = self.create_drive_path_estimator(self.obstacles)

        elif isinstance(edge, PushActionEdge):
            edge.path_estimator = self.create_push_path_estimator(self.get_node(edge.to).obstacle, self.obstacles)

        try:
            edge.path_estimator.search_path(
                self.get_node(edge.source).obstacle.state, self.get_node(edge.to).obstacle.state)
            edge.set_path_exist_status()

        except NoPathExistsException as exc:
            self.handle_no_path_exists_exception(exc, edge)

        if CREATE_SERVER_DASHBOARD:
            edge.path_estimator.visualise()

    def search_path(self, edge):
        """ Search for a path from start to target for an edge. """

        self.go_to_loop(SEARCHING_LOOP)
        # if a new edge is added (moving a obstacle to clear a path), a replanning of the hypothesis
        # happened. Copy the old hypothesis, add new edges an that is the new hypothesis. Store
        # the failed hypothesis in the logs

        assert isinstance(edge, ActionEdge), f"edge type must be ActionEdge and type is {type(edge)}"

        if edge.motion_planner is None:
            # motion planning
            if isinstance(edge, DriveActionEdge):
                edge.motion_planner = self.create_drive_motion_planner(self.obstacles, edge.path_estimator)
                current_state = self.robot.state

            elif isinstance(edge, PushActionEdge):

                edge.motion_planner = self.create_push_motion_planner(
                        self.obstacles, self.get_node(edge.source).obstacle, edge.path_estimator)
                current_state = self.get_node(edge.source).obstacle.state

        try:
            error_triggered = False
            (edge.path, add_node_list) = edge.motion_planner.search_path(current_state, self.get_node(edge.to).obstacle.state)

        except PlanningTimeElapsedException as exc:
            print('error wass triggered in path search')
            error_triggered = True
            add_node_list = []
            self.handle_planning_time_elapsed_exception(exc, edge)

        if error_triggered:
            return self.search_hypothesis()

        if CREATE_SERVER_DASHBOARD:
            self.visualise()
            edge.motion_planner.visualise()

        if isinstance(edge, PushActionEdge):
            self.create_drive_to_best_push_position_edge(edge)

        if len(add_node_list) > 0:
            print('add_node_list containts a thingy... first remove that obstacle mate')

            self.create_remove_obstacle_edge(add_node_list, edge)
            return


        if CREATE_SERVER_DASHBOARD:
            self.visualise()
            edge.motion_planner.visualise()

        edge.set_path_is_planned_status()

    ##############################################################################################
    ################ END MAIN FUNCTIONS ################# END MAIN FUNCTIONS #####################
    ##############################################################################################

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

        model_node = ObstacleNode(
                self.unique_node_iden(),
                self.robot.name+"_model",
                self.robot,
                self.get_node(edge.to).subtask_name)

        self.add_node(model_node)

        drive_ident_edge = DriveIdentificationEdge(iden=self.unique_edge_iden(),
                source=edge.source,
                to=model_node.iden,
                verb="identify",
                controller="controller", # identify edge, does not really need controller
                model_for_edge_iden=edge.iden,
                sys_model_name=edge.model_name)

        edge.source = model_node.iden
        drive_ident_edge.system_model = self.create_drive_model(edge.model_name)
        self.add_edge(drive_ident_edge)
        self.hypothesis.insert(self.edge_pointer, drive_ident_edge)

        self.current_edge=self.hypothesis[self.edge_pointer]
        print(f'now having current edge = {self.current_edge.iden}')

    def create_push_ident_edge(self, edge: PushActionEdge):
        """ create a system identification edge for pushing. """

        model_node = ObstacleNode(
                self.unique_node_iden(),
                self.get_node(edge.to).name+"_model",
                self.get_node(edge.to).obstacle,
                self.get_node(edge.to).subtask_name)

        self.add_node(model_node)

        push_ident_edge = PushIdentificationEdge(iden=self.unique_edge_iden(),
                source=edge.source,
                to=model_node.iden,
                verb="identify",
                controller="controller",
                model_for_edge_iden=edge.iden,
                sys_model_name=edge.model_name)

        edge.source = model_node.iden
        push_ident_edge.system_model = self.create_push_model(edge.model_name)
        self.add_edge(push_ident_edge)
        self.hypothesis.insert(self.edge_pointer, push_ident_edge)
        self.current_edge=self.hypothesis[self.edge_pointer]

    def create_drive_to_best_push_position_edge(self, edge: PushActionEdge):
        """ add edges/nodes to drive toward the best push position. """
        assert self.current_node is not None and self.current_edge is not None,\
                f"current_node: {self.current_node} and/or current_edge: {self.current_edge} cannot be None"

        # create and add best push pose against obstacle node
        best_push_pose_against_obstacle_state = self.find_push_pose_againts_obstacle_state(
                self.nodes[edge.to].obstacle,
                edge.path,
                edge.path_estimator)
        best_push_pose_against_obstacle_node = ObstacleNode(
                self.unique_node_iden(),
                "best_push_pose_against_"+edge.name,
                Obstacle(name=self.robot.name,
                    state=best_position_against_obstacle_state,
                    properties=self.robot.properties),
                edge.subtask_name)
        self.add_node(best_push_pose_against_obstacle_node)

        # rewire edge
        edge.source = best_push_pose_against_obstacle_node.iden

        # create driving edge
        self.create_drive_edge(edge.source, best_push_pose_against_obstacle_node)

    def create_remove_obstacle_edge(self, add_node_list: list, edge: ActionEdge):
        """ add edges/nodes to remove a obstacle. """

        # find which obstacle is blocking
        obst_keys = self.in_obstacle(add_node_list)

        if len(obst_keys) == 0:
            warnings.warn(f"could not find obstacle at {add_node_list[0]}, do not create edges to remove obstacle")
            return

        # only look at the first blocking obstacle, ignore others
        if len(obst_keys) > 1:
            warnings.warn(f"multiple obstacles are blocking, but only {obst_keys[0]} is moved")


        blocking_obst = self.obstacles[obst_keys[0]]

        # add obstacle start node if not yet present
        blocking_obst_start_node = None
        for temp_node in self.nodes:
            if temp_node.name == blocking_obst.name:
                print(f'here we should see the connection from emt edge to blocking object {temp_node.name}')
                blocking_obst_start_node = temp_node

        if blocking_obst_start_node is None:
            blocking_obst_start_node = ObstacleNode(
                self.unique_node_iden(),
                blocking_obst.name,
                blocking_obst,
                self.get_node(edge.to).subtask_name
                )
            self.add_node(blocking_obst_start_node)

        # rewire current node toward blocking obstacle start node
        self.add_edge(EmptyEdge(
            self.unique_edge_iden(),
            self.get_node(edge.source).iden,
            blocking_obst_start_node.iden))

        # find state that is not overlapping with planned path
        target_state = self.find_free_state_for_blocking_obstacle(blocking_obst, edge.motion_planner.shortest_path)

        # update existing target node if that already exist
        # blocking_obst_target_node = None
        # for temp_node in self.nodes:
        #     if temp_node.name == obst_keys[0]+"_target":
        #         print(f'updating exitingn node with status  {temp_node.status}')
        #         temp_node.state = target_state
        #         blocking_obst_target_node = temp_node
        #
        # # create new obstacle node
        # if blocking_obst_target_node is None:

        blocking_obst_target_node = ObstacleNode(
                self.unique_node_iden(),
                obst_keys[0]+"_target",
                Obstacle(obst_keys[0],
                    target_state,
                    blocking_obst.properties),
                self.get_node(edge.to).subtask_name)

        self.add_node(blocking_obst_target_node)
        edge.source = blocking_obst_target_node.iden

    @abstractmethod
    def create_drive_path_estimator(self, obstacles) -> PathEstimator:
        """ create drive path estimator. """

    @abstractmethod
    def create_push_path_estimator(self, push_obstacle, obstacles) -> PathEstimator:
        """ create push path estimator. """

    @abstractmethod
    def create_drive_motion_planner(self, obstacles, path_estimator):
        """ create drive motion planner. """

    @abstractmethod
    def create_push_motion_planner(self, obstacles):
        """ create push motion planner. """

    @abstractmethod
    def in_obstacle(self, pose_2ds: list) -> list:
        """ return the obstacle keys at pose_2ds. """

    @abstractmethod
    def find_push_pose_againts_obstacle_state(self, blocking_obst, path, path_estimator) -> State:
        """ return a starting state to start pushing the obstacle. """

    @abstractmethod
    def find_free_state_for_blocking_obstacle(self, blocking_obst: Obstacle, path: list) -> State:
        """ return a state where the obstacle can be pushed toward so it is not blocking the path. """

    def in_blacklist(self, edge_type: list) -> bool:
        """ checks if the edge is already in the blacklist. """

        if edge_type[0] in self.blacklist:
            for blacked_edge_type in self.blacklist[edge_type[0]]:
                if blacked_edge_type==edge_type:
                    return True
        return False

    def add_to_blacklist(self, edge):
        """ add edge to the blacklist. """

        assert edge.controller.system_model.name is not None,\
                "cannot add edge without system model name"
        assert callable(edge.controller.system_model.model),\
                "cannot add edge without callable system model"

        edge_type = [
                edge.to,
                self.get_node(edge.source).obstacle.name,
                type(edge),
                edge.controller.name,
                edge.controller.system_model.name]

        assert not self.in_blacklist(edge_type),\
                f"edge: {edge.iden} should not already be in blacklist"

        # create blacklist on target node key
        if edge.to in self.blacklist:
            self.blacklist[edge.to].append(edge_type)
        else:
            self.blacklist[edge.to] = [edge_type]


    #######################################################
    ############ HANDLING OF EXCEPTIONS ###################
    #######################################################
    def handle_running_out_of_control_methods_exception(self, exc: RunnoutOfControlMethodsException, target_node_iden: int):
        """ handle a RunnoutOfControlMethodsException. """

        self.get_node(target_node_iden).status = NODE_FAILED

        # make outgoing edges unfeasible and remove from hypothesis
        for temp_edge in self.get_outgoing_edges(target_node_iden):
            temp_edge.status = EDGE_FAILED
            if temp_edge in self.hypothesis:
                self.add_to_blacklist(temp_edge)
                self.fail_corresponding_iden_edge(temp_edge)
                self.hypothesis.remove(temp_edge)

        if LOG_METRICS:
            self.logger.add_failed_hypothesis(self.hypothesis, self.current_subtask, str(exc))
        if CREATE_SERVER_DASHBOARD:
            self.visualise()

    def handle_no_path_exists_exception(self, exc: NoPathExistsException, edge: Edge):
        """ handle a NoPathExistsException. """
        self.get_node(edge.to).status = EDGE_UNFEASIBLE
        edge.status = EDGE_FAILED
        self.fail_corresponding_iden_edge(edge)

        if LOG_METRICS:
            self.logger.add_failed_hypothesis(self.hypothesis, self.current_subtask, str(exc))
        if CREATE_SERVER_DASHBOARD:
            self.visualise()
            edge.path_estimator.visualise()

    def handle_planning_time_elapsed_exception(self, exc: PlanningTimeElapsedException, edge: Edge):
        """ handle a PlanningTimeElapsedException. """

        if LOG_METRICS:
            self.logger.add_failed_hypothesis(self.hypothesis, self.current_subtask, str(exc))

        self.add_to_blacklist(edge)
        edge.status = EDGE_FAILED
        self.fail_corresponding_iden_edge(edge)
        self.current_edge = None
        self.visualise()

        # remove all edges up to the failed edge from the current hypothesis.
        self.hypothesis = self.hypothesis[self.hypothesis.index(edge)+1:]
        self.edge_pointer = 0


    def handle_fault_detected_exception(self, exc: FaultDetectedException, edge: Edge):
        """ handle a FaultDetectedException. """
        pass


    #######################################################
    ### INCREMENTING THE EDGE AND KEEPING TRACK OF TIME ###
    #######################################################
    def fail_corresponding_iden_edge(self, edge: ActionEdge):
        """ update status of corresponding identification edge if it exists. """
        assert isinstance(edge, ActionEdge),\
                f"edge should be an ActionEdge but is {type(edge)}"

        for temp_edge in self.edges:
            if isinstance(temp_edge, IdentificationEdge) and temp_edge.model_for_edge_iden == edge.iden:
                print(f'found the corresponding ident edge, update status to failed')
                for outgoing_edge in self.get_outgoing_edges(temp_edge):
                    if isinstance(outgoing_edge, EmptyEdge):
                        outgoing_edge.status = EDGE_FAILED
                temp_edge.status = EDGE_FAILED

    def increment_edge(self):
        """ updates toward the next edge in the hypothesis. """

        # identification edge completed -> update a system model
        if isinstance(self.current_edge, IdentificationEdge):
            self.update_system_model(self.current_edge)

        # complete current edge
        self.current_edge.set_completed_status()

        # TODO: update KGraph

        if self.hypothesis_completed():
            # self.stop_timing()
            if LOG_METRICS:
                self.logger.add_succesfull_hypothesis(self.hypothesis, self.current_subtask)
            self.get_target_node(self.current_edge.to).status = NODE_COMPLETED
            self.reset_current_pointers()
            return self.search_hypothesis()

        print(f'incremnte edge from {self.current_edge.iden} to {self.hypothesis[self.edge_pointer+1]}')
        self.edge_pointer += 1

        # search_hypothesis checks the validity of the next edge
        self.search_hypothesis()

    def update_subtask(self) -> tuple:
        """ updates the current subtask, and returns the 2 unconnected nodes should be connected. """

        if self.current_subtask is None or self.current_subtask["target_node"].status != NODE_INITIALISED:

            # check if all tasks are completed
            # if all subtask are completed, conclude the task is completed, otherwise search new hypothesis
            if all(target_node.status != NODE_INITIALISED for target_node in self.target_nodes):

                n_completed_subtasks = 0
                n_failed_subtasks = 0
                n_unfeasible_subtasks = 0

                for target_node in self.target_nodes:
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

            # reset pointers
            self.reset_current_pointers()

            # find a new subtask
            unfinished_target_nodes = [target_node for target_node in self.target_nodes if target_node.status == NODE_INITIALISED]

            obstacle_target_node = None
            robot_target_node = None

            # first obstacle subtasks then robot target position
            for unfinished_target_node in unfinished_target_nodes:
                if unfinished_target_node.name == self.robot.name + "_target":
                    robot_target_node = unfinished_target_node
                else:
                    obstacle_target_node = unfinished_target_node

            if obstacle_target_node is None:
                if robot_target_node is None:
                    self.end_failed_task()
                    raise StopIteration("No more subtasks, No Solution Found!")

                subtask_start_node = self.get_start_node(self.get_start_iden_from_target_iden(robot_target_node.iden))
                subtask_target_node = robot_target_node
            else:
                subtask_start_node = self.get_start_node(self.get_start_iden_from_target_iden(obstacle_target_node.iden))
                subtask_target_node = obstacle_target_node

            self.current_subtask = {
                    "start_node": subtask_start_node,
                    "target_node": subtask_target_node,
                    "name": subtask_target_node.subtask_name,
                    "now_timing": None,
                    "start_time": None,
                    "execute_time": 0.0,
                    "search_time": 0.0,
                    }

            return (subtask_start_node, subtask_target_node)
        else:
            # focus on current subtask
            return self.find_nodes_to_connect_in_subtask()

    def find_nodes_to_connect_in_subtask(self) -> Tuple[Node, Node]:
        """ returns 2 nodes to connect in current subtask. """

        target_node = self.find_source_node(self.current_subtask["target_node"].iden)

        assert target_node.status == NODE_INITIALISED,\
                f"target node status must be {NODE_INITIALISED} and is {target_node.status} for node {target_node.iden}"
        start_node = self.find_corresponding_start_node(target_node)
        assert start_node.status == NODE_INITIALISED,\
                f"start node status must be {NODE_INITIALISED} and is {start_node.status} for node {target_node.iden}"

        return (start_node, target_node)

    def find_source_node(self, node_iden) -> Node:
        """ find the source node which points to this node via 0 or more edges.
        if a T junction is found (2 edges pointing to a node in the same subtask) an error is raised."""

        assert (self.get_node(node_iden).status == NODE_INITIALISED or\
                self.get_node(node_iden).status == NODE_COMPLETED), \
                f"node {node_iden} has status {self.get_node(node_iden).status}"
        assert self.current_subtask is not None,\
                "current subtask is none"

        if node_iden == ROBOT_IDEN:
            return self.robot_node

        edge_to_list = [edge for edge in self.edges if\
                edge.to == node_iden and\
                edge.status != EDGE_FAILED and\
                (self.get_node(edge.source) != NODE_UNFEASIBLE or\
                self.get_node(edge.source) != NODE_FAILED) and\
                self.get_node(edge.to).subtask_name == self.current_subtask["name"]]

        # no edges pointing toward this node -> return
        if len(edge_to_list) == 0:
            return self.get_node(node_iden)

        if len(edge_to_list) > 1:
            print(f"hey there are multiple edges now!:")
            self.visualise(save=False)

        # TODO: multiple edges cannot be pointing (at least not in the same subtask), do a dubble check
        assert not len(edge_to_list) > 1, f"multiple edges pointing toward with identifier {node_iden}."

        if edge_to_list[0].to == edge_to_list[0].source:
            edge = edge_to_list[0]
            print(f'I got u, u self loop: {edge.source} {edge.to}')

        assert not edge_to_list[0].to == edge_to_list[0].source, "self loop detected"

        # no fea
        if (self.get_node(edge_to_list[0].source).status == NODE_UNFEASIBLE or\
                self.get_node(edge_to_list[0].source).status == NODE_FAILED):


            return self.get_node(node_iden)
        else:
            return self.find_source_node(edge_to_list[0].source)

    def find_corresponding_start_node(self, target_node):
        """ find the obstacle node that should directly be connected to this node. """

        # check target obstacle, find the equivalent start node obstacle
        if target_node.obstacle.properties == self.robot.properties:
            return self.robot_node
        else:
            for temp_node in self.nodes:
                # TODO: if a obstacle is in the way (and auto generated as node) and a subtask, this might not succeed
                if temp_node.obstacle.properties == target_node.obstacle.properties and\
                        temp_node.iden != target_node.iden and\
                        target_node.subtask_name == self.current_subtask["name"]:
                    return temp_node

        # all corresponding nodes should be generated in setup function or remove_blocking_obstacle function
        raise ValueError(f'for target node {target_node.name} no start node was found')

    def find_push_state_against_obstacle(self, obstacle) -> State:
        """ find a push position (state) next to the obstacle. """
        if isinstance(obstacle.properties, BoxObstacle):
            pos = obstacle.state.pos

            # TODO: this is not generic enough, working now ha
            return State(pos=np.array([pos[0]+2.2, pos[1], pos[2]]))

            # TODO this for box

        if isinstance(obstacle.properties, (CylinderObstacle, SphereObstacle)):

            pos = obstacle.state.pos

            # TODO: this is not generic enough, working now ha
            return State(pos=np.array([pos[0]+2.2, pos[1], pos[2]]))

    def update_system_model(self, ident_edge):
        """ update system model of the next edge. """

        for_edge = self.get_edge(ident_edge.model_for_edge_iden)
        assert for_edge.status==EDGE_PATH_EXISTS,\
                f"edge status should be {EDGE_PATH_EXISTS} but is {for_edge.status}"
        system_model = ident_edge.system_model

        for_edge.set_has_system_model_status()

        if isinstance(for_edge.controller, DriveController):
            self.setup_drive_controller(for_edge.controller, system_model)
        elif isinstance(for_edge.controller, PushController):
            self.setup_push_controller(for_edge.controller, system_model, for_edge)
        else:
            raise ValueError(f"unknown controller of type {type(for_edge.controller)}")

    @abstractmethod
    def setup_drive_controller(self, controller, system_model):
        """ setup drive controller """

    @abstractmethod
    def setup_push_controller(self, controller, system_model, push_edge):
        """ setup push controller """

    def go_to_loop(self, loop: str):
        """ go to the searching loop. """

        if loop in [EXECUTION_LOOP, SEARCHING_LOOP]:

            self.in_loop = loop

            if LOG_METRICS:
                self.stop_timing()

                self.current_subtask["start_time"] = time.time()
                self.current_subtask["now_timing"] = loop

        else:
            raise ValueError(f"unknown loop: {loop}")

    def stop_timing(self):
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

###############################
### CREATION OF CONTROLLERS ###
###############################
    def create_drive_controller(self, target_iden: int) -> Tuple[Controller, str]:
        """ randomly select a driving controller that is not on the blacklist. """

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

    def filter_control_and_model_names(self, edge_type, control_and_models: list, target_iden: int) -> list:
        """ removes the controllers and edges that are on the blacklist from cntrol_and_models. """

        controller_and_model_names_filtered = []
        # filter out blacked combinations of control methods with system models

        if target_iden in self.blacklist:
            for (controller, model_names) in control_and_models:

                model_names_filtered = []
                for model_name in model_names:

                    if not self.in_blacklist([
                        target_iden,
                        self.get_node(target_iden).obstacle.name,
                        edge_type,
                        controller.name,
                        model_name]):
                        model_names_filtered.append(model_name)

                if len(model_names_filtered) > 0:
                    controller_and_model_names_filtered.append((controller, model_names_filtered))
        else:
            controller_and_model_names_filtered = control_and_models

        if len(controller_and_model_names_filtered) == 0:
            raise RunnoutOfControlMethodsException("All possible edges are on the blacklist")

        return controller_and_model_names_filtered

    @abstractmethod
    def get_drive_controllers(self) -> list:
        """ return drive controllers """

    @abstractmethod
    def create_drive_model(self, model_name: str) -> SystemModel:
        """ create the requested system model. """

    def create_push_controller(self, target_iden: int) -> Tuple[Controller, str]:
        """ create push controller. """

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

    @abstractmethod
    def get_push_controllers(self) -> list:
        """ get push controllers """

    @abstractmethod
    def create_push_model(self, model_name: str):
        """ create a dynamic model for pushing. """

    @abstractmethod
    def find_compatible_models(self, controllers: list) -> list:
        """ return compatible dynamic models for controllers. """
#####################################
### HYPOTHESIS CHECKING FUNCTIONS ###
#####################################

    def reset_current_pointers(self):
        """ resets the current node, edge and hypothesis. """
        self.current_subtask = None
        self.current_node = None
        self.current_edge = None
        self.hypothesis = []
        self.edge_pointer = 0

    def hypothesis_completed(self) -> bool:
        """ returns true if the hypothesis is completed, otherwise false. """
        return self.edge_pointer >= len(self.hypothesis)-1

    def is_current_subtask_connected(self) -> bool:
        """ check if the current subtask has a path from robot -> source node -> target node
        via non-failed edges. """
        if self.current_subtask is None:
            self.update_subtask()

        if self.is_reachable(self.current_subtask["start_node"].iden, self.current_subtask["target_node"].iden):
            if self.current_subtask["start_node"].iden == ROBOT_IDEN:
                return True
            elif self.is_reachable(ROBOT_IDEN, self.current_subtask["start_node"].iden):
                return True

        return False

    def is_reachable(self, source_node_iden, target_node_iden) -> bool:
        """ return true if there is a list of non-failed edges going from the start node
        identifier to target node identifier, otherwise return false. """
        assert isinstance(source_node_iden, int), f"source_node_iden should be an interger and is {type(source_node_iden)}"
        assert isinstance(target_node_iden, int), f"target_node_iden should be an interger and is {type(target_node_iden)}."

        reachable_from_start = [source_node_iden]

        while len(reachable_from_start) > 0:
            current_node_iden = reachable_from_start.pop(0)

            for outgoing_node_iden in self.point_toward_nodes(current_node_iden):
                if outgoing_node_iden == target_node_iden:
                    return True
                reachable_from_start.append(outgoing_node_iden)

        return False

    def get_start_node(self, iden) -> Node:
        """ return start node by id, raises error if the identifyer does not exist. """
        start_node_list = [node for node in self.start_nodes if node.iden == iden]

        if len(start_node_list) == 0:
            raise IndexError(f"a start node with identifyer {iden} does not exist.")

        return start_node_list[0]

    def get_target_node(self, iden) -> Node:
        """ return target node by id, raises error if the identifyer does not exist. """
        target_node_list = [node for node in self.target_nodes if node.iden  == iden]

        if len(target_node_list) == 0:
            raise IndexError(f"a target node with identifyer {iden} does not exist.")

        return target_node_list[0]

    def get_start_iden_from_target_iden(self, target_iden):
        assert isinstance(target_iden, int), f")target_iden should be an int and it {type(target_iden)}"
        start_iden_list = []
        for start_2_target in self.start_to_target_iden:
            if start_2_target[1] == target_iden:
                start_iden_list.append(start_2_target[0])

        if len(start_iden_list) > 1:
            raise ValueError(f"there are {len(start_iden_list)} start node identifiers from target node iden {target_iden}")
        else:
            return start_iden_list.pop(0)

    def get_target_idens_from_start_iden(self, start_iden) -> list:
        """ return list of target node identifiers. """

        assert isinstance(start_iden, int), f"start_iden should be an int and it is type {type(start_iden)}"
        target_idens_list = []
        for start_2_target in self.start_to_target_iden:
            if start_2_target[0] == start_iden:
                target_idens_list.append(start_2_target[1])

        return target_idens_list

    def end_completed_task(self, success_ratio):
        """ finalise logs when a task completed. """
        if LOG_METRICS:
            self.logger.complete_log_succes(success_ratio)
            self.logger.print_logs()
        if SAVE_LOG_METRICS:
            self.logger.save_logs()

    def end_failed_task(self):
        """ finalise logs when a task failed to complete. """
        if LOG_METRICS:
            self.logger.complete_log_failed()
            self.logger.print_logs()
        if SAVE_LOG_METRICS:
            self.logger.save_logs()

    def add_node(self, node):
        if isinstance(node, ChangeOfStateNode):
            raise TypeError("ChangeOfStateNode's are not allowed in HGraph")
        self.nodes.append(node)

    def add_start_node(self, node):
        if not isinstance(node, ObstacleNode):
            raise TypeError("ObstacleNode's are only allowed as starting node in HGraph")
        self.add_node(node)
        self.start_nodes.append(node)

    def add_target_node(self, node):
        if isinstance(node, ChangeOfStateNode):
            raise TypeError("ChangeOfStateNode's are not allowed as target node in HGraph")
        self.add_node(node)
        self.target_nodes.append(node)

    def visualise(self, save=True):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """

        net = Network(bgcolor=FIG_BG_COLOR, height="450px", directed=True)

        # set a custom style sheet
        net.path = PROJECT_PATH+"/dashboard/assets/graph_template.html"

        net.set_edge_smooth('dynamic')

        for node in self.start_nodes:
            if node == self.current_node:
                continue
            net.add_node(node.iden,
                    title = f"Starting Node: {node.name}<br>{node.to_string()}<br>",
                    x=10.0,
                    y=10.0,
                    label = node.name,
                    borderWidth= 1,
                    borderWidthSelected= 2,
                    color= {
                        'border': '#2B7CE9', # blue
                        'background': '#97C2FC',
                        'highlight': {
                            'border': '#2B7CE9',
                            'background': '#D2E5FF'
                            }
                        },
                    group = "start_nodes")

        for node in self.target_nodes:
            if node == self.current_node:
                continue
            net.add_node(node.iden,
                    title = f"Target Node: {node.name}<br>{node.to_string()}<br>",
                    x=10.0,
                    y=10.0,
                    label = node.name,
                    color= {
                        'border': '#009900', # green
                        'background': '#00ff00',
                        'highlight': {
                            'border': '#009900',
                            'background': '#99ff99'
                            }
                        },
                    group = "target_nodes")

        for node in self.nodes:
            if node == self.current_node:
                continue
            net.add_node(node.iden,
                    title = f"Node: {node.name}<br>{node.to_string()}<br>",
                    x=10.0,
                    y=10.0,
                    color= {
                        'border': '#ffa500', # yellow
                            'background': '#ffff00',
                        'highlight': {
                            'border': '#ffa500',
                            'background': '#ffff99'
                            }
                        },
                    label = node.name,
                    group = node.__class__.__name__)

        if self.current_node is not None:
            net.add_node(self.current_node.iden,
                    title = f"Current Node: {self.current_node.name}<br>{self.current_node.to_string()}<br>",
                    x=10.0,
                    y=10.0,
                    label = self.current_node.name,
                    color= {
                        'border': '#fb4b50',
                        'background': '#fb7e81',
                        'highlight': {
                            'border': '#fb4b50',
                            'background': '#fcbcc4'
                            }
                        },
                    #TODO this stuf
                    # ,label = self.current_node.verb,
                    group = "current_node")

        # add edges
        for edge in self.edges:

            value = 1.5
            if edge in self.hypothesis:
                value = 3

            if edge.status == EDGE_INITIALISED:
                color = "grey"
            elif edge.status == EDGE_COMPLETED:
                color = "green"
            elif edge.status == EDGE_FAILED:
                color = "red"
            else:
                color='black'

            # dashes = False
            # if edge.path is False:
            #     dashes = True

            net.add_edge(edge.source,
                    edge.to,
                    # dashes=dashes,
                    width=value,
                    color=color,
                    label=edge.verb,
                    title=f"{edge.to_string()}<br>",
                    )

        # if you want to edit cusomize the graph
        # net.show_buttons(filter_=['physics'])

        if save:
            net.write_html(name=PROJECT_PATH+"dashboard/data/hypothesis_graph.html")
        else:
            net.show("delete.html")

############################################
### SETTERS AND GETTERS BELOW THIS POINT ###
############################################
    @property
    def current_node(self):
        return self._current_node

    @current_node.setter
    def current_node(self, node):
        assert isinstance(node, (Node, type(None))), f"node should be a Node or None and is {type(node)}"
        self._current_node = node

    @property
    def current_edge(self):
        return self._current_edge

    @current_edge.setter
    def current_edge(self, edge):
        # if isinstance(edge, ActionEdge):
        #     assert isinstance(edge.motion_planner.shortest_path, list),\
        #             f"edge motion planner shortest path is not a list but {type(edge.motion_planner.shortest_path)}"
        # elif isinstance(edge, IdentificationEdge):
        #     # TODO: would you like sanitisation for an iden edge?
        #     pass
        # else:
        #     ValueError('error here')
        # more input sanitisation
        self._current_edge = edge

    @property
    def hypothesis(self):
        return self._hypothesis

    @hypothesis.setter
    def hypothesis(self, hyp):
        assert isinstance(hyp, list), f"hypothesis should be a list and is {type(hyp)}"
        assert all(isinstance(edge, Edge) for edge in hyp), "hypothesis should contain only edges"
        self._hypothesis = hyp

    @property
    def obstacles(self):
        return self._obstacles

    @obstacles.setter
    def obstacles(self, obstacles):
        assert isinstance(obstacles, dict),\
                f"obstacles should be a dictionary and is {type(obstacles)}"
        self._obstacles = obstacles

    @property
    def robot_order(self):
        return self._robot_order

    @robot_order.setter
    def robot_order(self, val):
        assert isinstance(val, int),\
                f"robot_order's type should be an int and is {type(val)}"
        assert val > 0, f"robot order should be higher than 0 and is {val}"
        self._robot_order = val
