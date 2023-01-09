from abc import abstractmethod
import random
from typing import Tuple
import numpy as np
from pyvis.network import Network

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.sphere_obstacle import SphereObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.global_planning.graph import Graph
from robot_brain.global_variables import FIG_BG_COLOR, COLORS, PROJECT_PATH, LOG_METRICS, CREATE_SERVER_DASHBOARD, SAVE_LOG_METRICS
# from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.node import Node
from robot_brain.global_planning.obstacle_node import ObstacleNode, COMPLETED, UNFEASIBLE, INITIALISED
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.obstacle import Obstacle
from robot_brain.state import State
from robot_brain.global_planning.drive_ident_edge import DriveIdentificationEdge
from robot_brain.global_planning.edge import Edge, INITIALISED, EXECUTING, FAILED
from robot_brain.global_planning.drive_act_edge import DriveActionEdge
from robot_brain.global_planning.push_ident_edge import PushIdentificationEdge
from robot_brain.global_planning.push_act_edge import PushActionEdge
from robot_brain.global_planning.action_edge import ActionEdge, PATH_EXISTS, PATH_IS_PLANNED, HAS_SYSTEM_MODEL
from robot_brain.global_planning.hgraph.local_planning.graph_based.configuration_grid_map import ConfigurationGridMap
from robot_brain.global_planning.identification_edge import IdentificationEdge
from robot_brain.global_planning.empty_edge import EmptyEdge
from robot_brain.controller.push.push_controller import PushController
from robot_brain.controller.drive.drive_controller import DriveController

from robot_brain.controller.controller import Controller
from logger.hlogger import HLogger

EXECUTION_LOOP = "executing"
SEARCHING_LOOP = "searching"
ROBOT_IDEN = 0

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

        # self.current_edge is the current edge being executed or first in line to be executed.
        # self.current_node is the source node of the current edge being executed for plotting purposes

        self.reset_current_pointers()
        if LOG_METRICS:
            self.logger = HLogger()

##########################################
### FUNCTIONS CREATING NODES AND EDGES ###
##########################################
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
            print(f'adding start to target {iden_start_node} to target {iden_target_node}')
            self.start_to_target_iden.append((iden_start_node, iden_target_node))

        for obst in obstacles:
            print('dkljfsdklf')
            print(obst)
            self.blacklist[obst] = []

        if CREATE_SERVER_DASHBOARD:
            self.visualise()
        if LOG_METRICS:
            self.logger.setup(task)

    def respond(self, current_state) -> np.ndarray:
        """ interface toward the robot simulation, eventually input for the robot is returned. """

        if self.in_loop == EXECUTION_LOOP:

            if len(self.hypothesis) == 0:
                raise ValueError("HGraph cannot respond without a hypothesis")

            # TODO: create function to set current edge, only check ready_for_execution once
            # self.current_edge = self.hypothesis[self.edge_pointer]

            # TODO: fault detection could be done here
            # TODO: when a fault is detected, abandon edge, move to blacklist and go to search loop

            if self.current_edge.view_completed(self.current_node.obstacle.state):
                if self.current_edge.completed():
                    self.current_edge.set_completed_status()

                    # identification edge completed -> update a system model
                    if isinstance(self.current_edge, IdentificationEdge):

                        self.update_system_model(self.current_edge)
                        # now set next edge then

                    if self.hypothesis_completed():
                        self.stop_timing()

                        if LOG_METRICS:
                            self.logger.add_succesfull_hypothesis(self.hypothesis, self.current_subtask)

                        # set all variables for completed hypothesis
                        # self.get_target_node(self.current_edge.to).completed = True

                        self.get_target_node(self.current_edge.to).status = COMPLETED

                        self.reset_current_pointers()

                        # go back to search loop
                        self._search_hypothesis()

                    else:
                        self.increment_edge()

                        return self.respond(current_state)

                else:
                    self.current_edge.increment_current_target()

            return self._edge_respond(self.current_edge, current_state)

        elif self.in_loop == SEARCHING_LOOP:
            self._search_hypothesis()
            return self.respond(current_state)

        else:
            raise ValueError(f"HGraph in an unknown loop: {self.in_loop}")

    def _edge_respond(self, edge: Edge, state: State) -> np.ndarray:
        """ edge returns control input to the robot. """
        if isinstance(edge, PushActionEdge):
            obst_state = self.get_node(edge.source).obstacle.state
            return edge.respond(state, obst_state=obst_state)
        elif isinstance(edge, DriveActionEdge):
            return edge.respond(state)
        elif isinstance(edge, IdentificationEdge):
            return edge.respond(state)
        else:
            raise ValueError(f"edge type DriveAction or PushAction Edge expected and is {type(edge)}")


    def _search_hypothesis(self):
        """ Search by backward induction from a target node toward the robot start node. """

        # find nodes in new subtask or current unfinished subtask
        (start_node, target_node) = self.update_subtask()
        print(f'start node name {start_node.name} target anem {target_node.name}')

        # TODO: find if start node and target node are: (start robot, target robot) or (start obstacle, target obstacle)

        self.go_to_loop(SEARCHING_LOOP)

        # search for unfinished target node and connect to starting node
        # TODO: hypothesis should be reachable, but also the first edge in hypothesis (or the first after the current edge) should be
        while not self.is_reachable(ROBOT_IDEN, self.current_subtask["target_node"].iden):

            # the obstacles should be the same between an edge
            assert start_node.obstacle.name == target_node.obstacle.name,\
            f"obstacle: {start_node.name} in start_node should be equal to obstacle: {target_node.name} in target_node"

            # driving action
            if start_node.iden == ROBOT_IDEN:
                self.create_drive_edge(start_node.iden, target_node.iden)

            # pushing action
            elif start_node.iden != ROBOT_IDEN:
                self.create_push_edge(start_node.iden, target_node.iden)

            else:
                raise ValueError("start and target nodes should be both the robot or both an obstacle")

            # find_subtask is not forced to find the same final_target_node
            (start_node, target_node) = self.find_nodes_to_connect_in_subtask()

        self.current_edge = self.hypothesis[0]
        self.current_node = self.get_node(self.current_edge.source)

        # check if the first edge is planned.
        if not self.current_edge.ready_for_execution():
            # TODO: check why the edge is not ready for execution, then fix according to that

            # check if the first edge to execute is planned if it is an ActionEdge
            if isinstance(self.current_edge, ActionEdge):
                if self.current_edge.status == HAS_SYSTEM_MODEL:
                    self.search_path(self.current_edge, False)

        self.current_edge.set_executing_status()
        self.go_to_loop(EXECUTION_LOOP)
        if CREATE_SERVER_DASHBOARD:
            self.visualise()

    def update_subtask(self) -> tuple:
        """ updates the current subtask, and returns the 2 unconnected nodes should be connected. """

        if self.current_subtask is None or self.current_subtask["target_node"].status != INITIALISED:

            # check if all tasks are completed
            # if all subtask are completed, conclude the task is completed, otherwise search new hypothesis
            if all(target_node.status != INITIALISED for target_node in self.target_nodes):

                n_completed_subtasks = 0
                n_failed_subtasks = 0

                for target_node in self.target_nodes:
                    if target_node.status == COMPLETED:
                        n_completed_subtasks += 1
                    elif target_node.status == FAILED:
                        n_failed_subtasks += 1
                    else:
                        raise ValueError(f"target node status should be {FAILED} or {COMPLETED} and is {target_node.status}")

                if n_failed_subtasks+n_completed_subtasks == 0:
                    task_success_ratio = None
                else:
                    task_success_ratio = n_completed_subtasks/(n_failed_subtasks+n_completed_subtasks)

                self.end_completed_task(task_success_ratio)

                raise StopIteration(f"The task is successfully completed with a success/fail ration of {task_success_ratio}!")

            # reset pointers
            self.reset_current_pointers()

            # find a new subtask
            unfinished_target_nodes = [target_node for target_node in self.target_nodes if target_node.status == INITIALISED]

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

                else:
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
        assert target_node.status == INITIALISED,\
                f"target node status must be {INITIALISED} and is {target_node.status}"
        start_node = self.find_corresponding_start_node(target_node)
        assert start_node.status == INITIALISED,\
                f"start node status must be {INITIALISED} and is {start_node.status}"

        return (start_node, target_node)


    def find_source_node(self, node_iden) -> Node:
        """ find the source node which points to this node via 0 or more edges.
        if a T junction is found (2 edges pointing to a node) an error is raised."""

        assert self.current_subtask is not None,\
                "current subtask is none"

        edge_to_list = [edge for edge in self.edges if\
                edge.to == node_iden and\
                edge.status != FAILED and\
                self.get_node(edge.to).subtask_name == self.current_subtask["name"]]

        if len(edge_to_list) == 0:
            return self.get_node(node_iden)
        elif self.current_node == self.get_node(node_iden):
            return self.current_node
        else:
            assert not len(edge_to_list) > 1, f"multiple edges pointing toward with identifier {node_iden}."

            return self.find_source_node(edge_to_list[0].source)

    def find_corresponding_start_node(self, target_node):
        """ find the obstacle node that should directly be connected to this node. """

        # check target obstacle, find the equivalent start node obstacle
        if target_node.obstacle.properties==self.robot.properties:
            return self.robot_node
        else:
            for temp_node in self.nodes:
                # TODO: if a obstacle is in the way (and auto generated as node) and a subtask, this might not succeed
                if temp_node.obstacle.properties == target_node.obstacle.properties and\
                        temp_node.iden != target_node.iden and\
                        target_node.subtask_name == self.current_subtask["name"]:
                    return temp_node

        # TODO: acceptional error if the start node is not yet created, future work mister, gijs groote 28 dec 2022
        # TODO: create new node here!, with the current position of the obstacle
        raise ValueError(f'for target node {target_node.name} no start node was found')

    def create_drive_edge(self, start_node_iden: int, target_node_iden: int):
        """ returns create drive edge and adds created model node to hgraph. """

        knowledge_graph = False

        if knowledge_graph:
            print("knowledge graph has a proposition")
            # TODO: the knowledge graph should come into play here
        else:

            controller = self.create_drive_controller()

            model_node = ObstacleNode(self.unique_node_iden(),
                    self.robot.name+"_model", self.robot,
                    self.nodes[target_node_iden].subtask_name)

            self.add_node(model_node)

            edge = DriveActionEdge(iden=self.unique_edge_iden(),
                    source=model_node.iden,
                    to=target_node_iden,
                    verb="driving",
                    controller=controller)

            try:
                self.estimate_path(edge)
            except ValueError as exc:
                # path estimation fails
                # TODO: unfeasible status does not prevent the node, or edge from occuring
                self.get_node(edge.to).status = UNFEASIBLE
                edge.status = FAILED
                print(f'reason for failing hypothesis, no path between start and target was found, reason {exc}') # <--- delete that please

                if LOG_METRICS:
                    self.logger.add_failed_hypothesis(self.hypothesis, self.current_subtask, str(exc))

                # TODO: is it okey to loop into this all the time
                return self._search_hypothesis()

            edge.set_path_exist_status()
            self.add_edge(edge)
            self.hypothesis.insert(0, edge)

            # TODO: if creation of the drive edge fails, it tries again, but can eventually throw an error. Then
            # go back to the drawing board
            try:
                self.create_drive_ident_edge(start_node_iden, model_node.iden, edge.iden)
            except ValueError as exc:
                raise exc
                # TODO: fail this edge, add to blacklist and try again
                # TODO: define behavioru when all sys iden methods fail.

    def create_drive_ident_edge(self, start_node_iden: int, target_node_iden: int, model_for_edge_iden: int):
        """ create a system identification edge. """

        model_for_node_controller_type = self.get_edge(model_for_edge_iden).controller.name # temp fix

        # TODO: check for blacklist which sys iden methods are available

        if model_for_node_controller_type == "MPC":
            controller = self._create_mpc_drive_controller()
        elif model_for_node_controller_type == "MPPI":
            controller = self._create_mppi_drive_controller()
        else:
            raise ValueError("somethign goes terebly wrong")

        edge = DriveIdentificationEdge(iden=self.unique_edge_iden(),
                source=start_node_iden,
                to=target_node_iden,
                verb="identify",
                controller=controller,
                model_for_edge_iden=model_for_edge_iden)

        try:
            edge.system_model= self.create_drive_model(controller.name)
            self.add_edge(edge)
            self.hypothesis.insert(0, edge)

        except ValueError as exc:
            # system identification failed

            # TODO: this iden method should be excluded for this obstacle, add to blacklist
            edge.status = FAILED
            if LOG_METRICS:
                self.logger.add_failed_hypothesis(self.hypothesis, self.subtask, str(exc))
                self.create_drive_ident_edge(start_node_iden, target_node_iden, model_for_edge_iden)


    def create_push_edge(self, start_node_iden: int, target_node_iden: int):
        """ returns create push edge and adds created model node to hgraph. """

        knowledge_graph = False

        if knowledge_graph:
            print("knowledge graph has a proposition")
            # TODO: the knowledge graph should come into play here
        else:

            controller = self.create_push_controller()

            start_node = self.nodes[start_node_iden]
            target_node = self.nodes[target_node_iden]

            model_node = ObstacleNode(self.unique_node_iden(), start_node.name+"_model", start_node.obstacle, target_node.subtask_name)
            self.add_node(model_node)

            edge = PushActionEdge(iden=self.unique_edge_iden(),
                    source=model_node.iden,
                    to=target_node_iden,
                    verb="pushing",
                    controller=controller)

            try:
                self.estimate_path(edge)
            except ValueError as exc:
                # path estimation fails
                self.get_target_node(edge.to).status = UNFEASIBLE
                edge.status = FAILED

                # TODO: give info to terminal for every failed thingy
                print(f"Failed hypothesis for subtask {target_node.subtask_name}, Path estimation failed: {exc}")

                if LOG_METRICS:
                    self.logger.add_failed_hypothesis(self.hypothesis, self.current_subtask, str(exc))

                return self._search_hypothesis()

            edge.set_path_exist_status()
            self.add_edge(edge)
            self.hypothesis.insert(0, edge)

            # TODO: if creation of the push edge fails, it tries again, but can eventually throw an error. Then
            # go back to the drawing board
            try:
                self.create_push_ident_edge(start_node_iden, model_node.iden, edge.iden)
            except ValueError as exc:
                raise exc
                # TODO: fail this edge, add to blacklist and try again
                # TODO: define behavioru when all sys iden methods fail.

            # create a node to drive the robot toward the box.
            target = self.find_push_state_against_obstacle(self.get_node(start_node_iden).obstacle)
            robot_to_box_id = self.unique_node_iden()
            robot_target_node = ObstacleNode
            self.add_node(ObstacleNode(
                robot_to_box_id,
                self.robot.name+"_to_"+self.get_node(start_node_iden).name,
                Obstacle(self.robot.name, target, self.robot.properties),
                self.get_node(target_node_iden).subtask_name
                ))


            self.add_edge(EmptyEdge(self.unique_edge_iden(), robot_to_box_id, start_node_iden))


    def create_push_ident_edge(self, start_node_iden: int, target_node_iden: int, model_for_edge_iden: int):
        """ create a system identification edge for pushing. """

        model_for_node_controller_type = self.get_edge(model_for_edge_iden).controller.name # temp fix

        # TODO: check for blacklist which sys iden methods are available

        if model_for_node_controller_type == "MPPI":
            controller = self._create_mppi_push_controller()
        else:
            raise ValueError("somethign goes terebly wrong")

        edge = PushIdentificationEdge(iden=self.unique_edge_iden(),
                source=start_node_iden,
                to=target_node_iden,
                verb="identify",
                controller=controller,
                model_for_edge_iden=model_for_edge_iden)

        try:
            system_model = self.create_push_model(controller.name) # todo, system iden
            edge.system_model = system_model
            self.add_edge(edge)
            self.hypothesis.insert(0, edge)

        except ValueError as exc:
            # system identification failed
            print(f"System Identification failed: {exc} in subtask {self.nodes[target_node_iden].subtask_name}, ")

            # TODO: this iden method should be excluded for this obstacle, add to blacklist
            edge.status = FAILED
            if LOG_METRICS:
                self.logger.add_failed_hypothesis(self.hypothesis, self.nodes[target_node_iden].subtask_name , str(exc))
                self.create_push_ident_edge(start_node_iden, target_node_iden, model_for_edge_iden)

    def find_push_state_against_obstacle(self, obstacle) -> State:
        """ find a push position (state) next to the obstacle. """
        if isinstance(obstacle.properties, BoxObstacle):
            pos = obstacle.state.pos

            # TODO: this is not generic enough, working now ha
            return State(pos=np.array([pos[0]+2.2, pos[1], pos[2]]))

            # TODO this for box

        if isinstance(obstacle.properties, (CylinderObstacle, SphereObstacle)):
            print('you did not implement this mister mister;')
            # TODO: this for cylinder and sphere

    def in_blacklist(self, edge) -> bool:
        """ checks if the edge is already in the blacklist. """

        # TODO: how to split the model present and not present in edge?
        if edge.system_model is not None:
            sys_model_name = edge.system_model.name
        else:
            sys_model_name = None

        for edge_type in self.blacklist[self.get_node(edge.source).obstacle.name]:
            # check if edge type is equal to the type of edge
            if edge_type[0]==type(edge) and edge_type[1]==edge.controller.name and edge_type[2]==sys_model_name:
                return True

        return False

    def add_to_blacklist(self, edge):
        """ add edge to the blacklist. """

        if edge.system_model is not None:
            sys_model_name = edge.system_model.name
        else:
            sys_model_name = None

        edge_type = (type(edge), edge.controller.name, sys_model_name)

        self.blacklist[self.get_node(edge.source).obstacle.name].append(edge_type)

###########################################
### PATH ESTIMATION AND MOTION PLANNING ###
###########################################

    # @abstractmethod
    # def estimate_robot_path_existance(self, target_state, obstacles):
    #     pass
    #
    # def estimate_path_existance(self, edge: ActionEdge) -> bool:
    #     """ todo graph based path existence """
    #     pass
    #

    def estimate_path(self, edge):
        """ Estimate path existance for from start to target for an edge. """

        self.go_to_loop(SEARCHING_LOOP)

        assert isinstance(edge, ActionEdge), f"edge type must be ActionEdge and type is {type(edge)}"

        # path estimation
        if isinstance(edge, DriveActionEdge):
            edge.path_estimator = self.create_drive_path_estimator(self.obstacles)

        elif isinstance(edge, PushActionEdge):
            edge.path_estimator = self.create_push_path_estimator(self.get_node(edge.to).obstacle, self.obstacles)

        (path_estimation, does_path_exist) = edge.path_estimator.search_path(self.get_node(edge.source).obstacle.state, self.get_node(edge.to).obstacle.state)

        if CREATE_SERVER_DASHBOARD:
            edge.path_estimator.visualise()

        if does_path_exist:
            edge.path_estimation = path_estimation
        else:
            # self.add_to_blacklist(edge)
            raise ValueError("path does not exist")


    @abstractmethod
    def create_drive_path_estimator(self, obstacles) -> ConfigurationGridMap:
        pass

    @abstractmethod
    def create_push_path_estimator(self, push_obstacle, obstacles) -> ConfigurationGridMap:
        pass

    def search_path(self, edge, increment_edge: bool):
        """ Search for a path from start to target for an edge. """

        self.go_to_loop(SEARCHING_LOOP)
        # if a new edge is added (moving a obstacle to clear a path), a replanning of the hypothesis
        # happened. Copy the old hypothesis, add new edges an that is the new hypothesis. Store
        # the failed hypothesis in the logs

        assert isinstance(edge, ActionEdge), f"edge type must be ActionEdge and type is {type(edge)}"

        # motion planning
        if isinstance(edge, DriveActionEdge):
            edge.motion_planner = self.create_drive_motion_planner(self.obstacles, edge.path_estimation)
            current_state = self.robot.state

        elif isinstance(edge, PushActionEdge):
            edge.motion_planner = self.create_push_motion_planner(self.obstacles, self.get_node(edge.source).obstacle, edge.path_estimator)
            print(f'the motion planner is hrere {type(edge.motion_planner)}')
            current_state = self.get_node(edge.source).obstacle.state

        try:
            edge.path = edge.motion_planner.search_path(current_state, self.get_node(edge.to).obstacle.state)
        except StopIteration as exc:

            if LOG_METRICS:
                self.logger.add_failed_hypothesis(self.hypothesis, self.current_subtask, str(exc))


            for temp_edge in self.hypothesis:
                print(f"edge name:{temp_edge.iden} and status {temp_edge.status}")

            print(f"Motion Planning failed: {exc} in subtask: {self.nodes[edge.to].subtask_name}")

            # self.add_to_blacklist(edge)
            edge.status = FAILED
            self.current_edge = None
            self.visualise()
            # remove all edges up to the failed edge from the current hypothesis.
            self.hypothesis = self.hypothesis[self.hypothesis.index(edge)+1:]


            self.edge_pointer = 0
            for tedge in self.hypothesis:
                print(f"new hypoth:{tedge.iden} and status {tedge.status}")

            return self._search_hypothesis()

        edge.set_path_is_planned_status()

        if increment_edge:
            self._force_increment_edge(edge)

        if CREATE_SERVER_DASHBOARD:
            edge.motion_planner.visualise()

    @abstractmethod
    def create_drive_motion_planner(self, obstacles, path_estimator):
        pass

    @abstractmethod
    def create_push_motion_planner(self, obstacles):
        pass

#######################################################
### INCREMENTING THE EDGE AND KEEPING TRACK OF TIME ###
#######################################################

    def increment_edge(self):
        """ updates toward the next edge in the hypothesis. """

        next_edge = self.hypothesis[self.edge_pointer+1]
        print(f'next edge iden {next_edge.iden}')

        # plan path if not yet planned
        if isinstance(next_edge, ActionEdge):
            if next_edge.status == HAS_SYSTEM_MODEL:
                self.search_path(next_edge, True)
                print('go back to driving and stuff call incrment edge again!')
                return

        # TODO: what if planning a path results in a subtask that should first be taken care of?

        for temp_edge in self.hypothesis:
            print(f"in increment edgee name:{temp_edge.iden} and status {temp_edge.status}")

        self._force_increment_edge(next_edge)

    def _force_increment_edge(self, next_edge):
        """ forces incrementing to the next edge """
        next_node = self.get_node(next_edge.to)

        # assert
        assert isinstance(next_edge, Edge),\
                f"next edge should be of type edge and is type {type(next_edge)}"
        assert next_edge.ready_for_execution(),\
                f"cannot increment toward next edge of type {type(next_edge)}"\
                "in hypothesis, edge is not ready"
        assert next_node.ready_for_execution(),\
                "next node is not ready for execution"
        # add ghost pose
        if isinstance(next_edge, PushActionEdge):
            self.env.add_target_ghost(next_node.obstacle.properties.name(),
                    next_node.obstacle.state.get_2d_pose())

        # increment edge
        self.edge_pointer += 1
        next_edge.set_executing_status()
        self.current_edge = next_edge
        self.current_node = self.get_node(self.hypothesis[self.edge_pointer].source)

        if CREATE_SERVER_DASHBOARD:
            self.visualise()
        self.go_to_loop(EXECUTION_LOOP)


    def update_system_model(self, ident_edge):
        """ update system model of the next edge. """

        for_edge = self.get_edge(ident_edge.model_for_edge_iden)
        assert for_edge.status==PATH_EXISTS,\
                f"edge status should be {PATH_EXISTS} but is {for_edge.status}"
        system_model = ident_edge.system_model

        for_edge.set_has_system_model_status()

        if isinstance(for_edge.controller, DriveController):
            self._setup_drive_controller(for_edge.controller, system_model)
        elif isinstance(for_edge.controller, PushController):
            self._setup_push_controller(for_edge.controller, system_model, for_edge)
        else:
            raise ValueError(f"unknown controller of type {type(for_edge.controller)}")

    @abstractmethod
    def _setup_drive_controller(self, controller, system_model):
        """ TODO: create this. """

    @abstractmethod
    def _setup_push_controller(self, controller, system_model, push_edge):
        """ TODO: create this. """

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
    def create_drive_controller(self):
        """for now: randomly select a driving controller. """
        # TODO: find banned controllers, find blacklist, ask Kgraph for advice,
        # fallback option is random select over all the availeble controllers

        possible_controllers = self.get_drive_controllers()

        controller = random.choice(possible_controllers)()

        return controller

    @abstractmethod
    def get_drive_controllers(self) -> list:
        """ return drive controllers """

    @abstractmethod
    def create_drive_model(self, controller_name: str):
        """ create a dynamic model for driving. """

    def create_push_controller(self) -> Controller:
        possible_controllers = self.get_push_controllers()
        return random.choice(possible_controllers)()

    @abstractmethod
    def get_push_controllers(self) -> list:
        """ get push controllers """

    @abstractmethod
    def create_push_model(self, controller_name: str):
        """ create a dynamic model for pushing. """

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

    def is_reachable(self, start_node_iden, target_node_iden) -> bool:
        """ return true if there is a list of non-failed edges going from the start node
        identifier to target node identifier, otherwise return false. """
        assert isinstance(start_node_iden, int), f"start_node_iden should be an interger and is {type(start_node_iden)}"
        assert isinstance(target_node_iden, int), f"target_node_iden should be an interger and is {type(target_node_iden)}."

        reachable_from_start = [start_node_iden]

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
        assert isinstance(target_iden, int), f"target_iden should be an int and it {type(target_iden)}"
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

    def visualise(self, save=bool):
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
                    title = f"Current Node: {node.name}<br>{node.to_string()}<br>",
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

            if edge.status==INITIALISED:
                color = "grey"
            elif edge.status==COMPLETED:
                color = "green"
            elif edge.status==FAILED:
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
        if isinstance(edge, ActionEdge):
            assert isinstance(edge.path, list), f"edge path is not a list but {type(edge.path)}"
        elif isinstance(edge, IdentificationEdge):
            # TODO: would you like sanitisation for an iden edge?
            pass
        else:
            ValueError('error here')
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
