# import pandas as pCOMPLETED d

from abc import abstractmethod
import numpy as np
import random
from pyvis.network import Network
from typing import Tuple
from robot_brain.global_planning.graph import Graph
from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH, LOG_METRICS, CREATE_SERVER_DASHBOARD, SAVE_LOG_METRICS

from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.node import Node
from robot_brain.global_planning.obstacle_node import ObstacleNode, COMPLETED, UNFEASIBLE, INITIALISED
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.obstacle import Obstacle
from robot_brain.global_planning.drive_ident_edge import DriveIdentificationEdge
from robot_brain.global_planning.drive_act_edge import DriveActionEdge
from robot_brain.global_planning.push_ident_edge import PushIdentificationEdge
from robot_brain.global_planning.push_act_edge import PushActionEdge
from robot_brain.global_planning.action_edge import ActionEdge, PATH_IS_PLANNED, HAS_SYSTEM_MODEL, FAILED
from robot_brain.global_planning.hgraph.local_planning.graph_based.configuration_grid_map import ConfigurationGridMap
from robot_brain.global_planning.identification_edge import IdentificationEdge 


from robot_brain.controller.controller import Controller 
import time
from robot_brain.state import State
from logger.hlogger import HLogger

EXECUTION_LOOP = "executing"
SEARCHING_LOOP = "searching"
ROBOT_IDEN = 0

class HGraph(Graph):
    """
    Hypothesis graph.
    """
    def __init__(self):
        Graph.__init__(self)
        
        self.in_loop = SEARCHING_LOOP               # hgraph can be in the execution or search loop
        self.task = None                            # task in form [(obst_name, target_state),..]
        self.robot_node = None                      # node containing the robot itself
        self.obstacles = {}                         # dictionary with all obstacles
        self.start_nodes = []                       # starting nodes one for every subtask and one for the robot
        self.target_nodes = []                      # target nodes one for every subtask
        self.start_to_target_iden = []              # identifier mapping from start to target node and vise versa


        # self.current_edge is the current edge being executed or first in line to be executed.
        # self.current_node is the source node of the current edge being executed for plotting purposes

        self.current_subtask = None                 # subtask in spotlight, robot 'thinks' and executes to complete this subtask
        self.current_node = None
        self.hypothesis = []                        # task sequence to complete a subtask in form [edge1, edge2, ...]
        self.edge_pointer = 0                       # pointer for the current edge in hypothesis
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
        
        # For every task create a start and target node
        for (subtask_name, (obst_temp, target)) in task.items():
            iden_start_node = 0 # robots unique id
            if obst_temp != self.robot:
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

            if self.current_edge.view_completed(current_state):

                if self.current_edge.completed():
                    self.current_edge.set_completed_status()

                    # identification edge completed -> update a system model
                    if isinstance(self.current_edge, IdentificationEdge):
                        self.update_system_model(self.current_edge)

                    if self.hypothesis_completed():
                        self.stop_timing()

                        if LOG_METRICS:
                            self.logger.add_succesfull_hypothesis(self.hypothesis, self.current_subtask)

                        # set all variables for completed hypothesis
                        # self.get_target_node(self.current_edge.to).completed = True
                        self.get_target_node(self.current_edge.to).status = COMPLETED

                        # go back to search loop
                        self.search_hypothesis()

                    else:
                        self.increment_edge()

                        return self.respond(current_state)

                else:
                    self.current_edge.increment_current_target()
            
            return self.current_edge.respond(current_state)

        elif self.in_loop == SEARCHING_LOOP:
            self.search_hypothesis()
            return self.respond(current_state)

        else:
            raise ValueError(f"HGraph in an unknown loop: {self.in_loop}")

    def search_hypothesis(self):
        """ Search by backward induction from a target node toward the robot start node. """

        # find nodes in new subtask or current unfinished subtask
        (start_node, target_node) = self.update_subtask()

        # TODO: find if start node and target node are: (start robot, target robot) or (start obstacle, target obstacle)

        self.go_to_loop(SEARCHING_LOOP)
        
        # search for unfinished target node and connect to starting node
        # TODO: hypothesis should be reachable, but also the first edge in hypothesis (or the first after the current edge) should be 
        # executable ( is planning, system ident and evertyhign done)
        while not self.is_reachable(ROBOT_IDEN, self.current_subtask["target_node"].iden):


            # the obstacles should be the same between an edge
            assert start_node.obstacle.name == target_node.obstacle.name, f"obstacle: {start_node.name} in start_node should be equal to obstacle: {target_node.name} in target_node"
            assert target_node.iden in self.get_target_idens_from_start_iden(start_node.iden), "there exist no edge from start_node pointing to target_node"

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
        
        # check if the first edge to execute is planned if it is an ActionEdge
        if isinstance(self.current_edge, ActionEdge):
            if self.current_edge.status == HAS_SYSTEM_MODEL:
                self.search_path(self.current_edge)

        # check if the first edge is planned. 
        if not self.current_edge.ready_for_execution():
            # TODO: check why the edge is not ready for execution, then fix according to that
            self.search_path(self.current_edge)

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



            # with a new subtask, emtpy the current hypothesis and current node
            self.hypothesis = []
            self.current_node = None
            self.edge_pointer = 0
            self.current_edge = None
            self.current_subtask = None

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
        # TODO: this function could return start position of the robot twice!, fix that
        """ returns 2 nodes to connect in current subtask. """
        
        if self.current_node is None:
            start_node = self.current_subtask["start_node"]
        else:
            start_node = self.current_node # make sure every new subtask sets the current node to None

        target_node = self.find_source_node(self.current_subtask["target_node"].iden)

        return (start_node, target_node)

    def create_drive_edge(self, start_node_iden: int, target_node_iden: int):
        """ returns create drive edge and adds created model node to hgraph. """

        knowledge_graph = False

        if knowledge_graph:
            print("knowledge graph has a proposition")
            # TODO: the knowledge graph should come into play here
        else:
            
            controller = self.create_drive_controller()

            model_node = ObstacleNode(self.unique_node_iden(), self.robot.name+"_model", self.robot, self.nodes[target_node_iden].subtask_name)
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
                self.get_target_node(edge.to).status = UNFEASIBLE
                edge.status = FAILED
                print(f'reason for failing hypothesis, no path between start and target was found, reason {exc}') # <--- delete that please

                if LOG_METRICS:
                    self.logger.add_failed_hypothesis(self.hypothesis, self.current_subtask, str(exc))

                return self.search_hypothesis()

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
            dyn_model = self.create_drive_model(controller.name) # todo, system iden
            edge.dyn_model = dyn_model
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

                return self.search_hypothesis()

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
            dyn_model = self.create_push_model(controller.name) # todo, system iden
            edge.dyn_model = dyn_model
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
            edge.path_estimator = self.create_push_path_estimator(self.obstacles)

        (path_estimation, does_path_exist) = edge.path_estimator.shortest_path(self.get_node(edge.source).obstacle.state, self.get_node(edge.to).obstacle.state)

        if CREATE_SERVER_DASHBOARD:
            edge.path_estimator.visualise()

        if does_path_exist:
            edge.path_estimation = path_estimation
        else:
            raise ValueError("path does not exist")


    @abstractmethod
    def create_drive_path_estimator(self, obstacles) -> ConfigurationGridMap:
        pass

    @abstractmethod
    def create_push_path_estimator(self, obstacles):
        pass

    def search_path(self, edge):
        """ Search for a path from start to target for an edge. """

        self.go_to_loop(SEARCHING_LOOP)
        # if a new edge is added (moving a obstacle to clear a path), a replanning of the hypothesis 
        # happened. Copy the old hypothesis, add new edges an that is the new hypothesis. Store
        # the failed hypothesis in the logs

        assert isinstance(edge, ActionEdge), f"edge type must be ActionEdge and type is {type(edge)}"

        path = []
        does_path_exist = False

        # motion planning
        if isinstance(edge, DriveActionEdge):
            edge.motion_planner = self.create_drive_motion_planner(self.obstacles)

        elif isinstance(edge, PushActionEdge):
            edge.motion_planner = self.create_push_motion_planner(self.obstacles)

        try:
            # TODO: no behavior is defined for an obstacle in the way
            edge.path = edge.motion_planner.search(self.robot.state, self.get_node(edge.to).obstacle.state)
        except StopIteration as exc:

            print(f"Motion Planning failed: {exc} in subtask: {self.nodes[edge.to].subtask_name}")

            # TODO: add this edge to blacklist for this obstacle
            edge.status = FAILED
            if LOG_METRICS:
                self.logger.add_failed_hypothesis(self.hypothesis, self.current_subtask, str(exc))
            return self.search_hypothesis()

        edge.set_path_is_planned_status()

    @abstractmethod
    def create_drive_motion_planner(self, obstacles):
        pass

    @abstractmethod
    def create_push_motion_planner(self, obstacles):
        pass

#######################################################
### INCREMENTING THE EDGE AND KEEPING TRACK OF TIME ###
#######################################################

    def increment_edge(self):
        """ updates toward the next edge in the hypothesis. """

        self.edge_pointer = self.edge_pointer + 1
        next_current_edge = self.hypothesis[self.edge_pointer]
        # check if the next edge is ready
        if next_current_edge.ready_for_execution():

            print('this edge is ready for execution')
        else:
            print('this edge is not yet ready for exectuion')
            if next_current_edge.status == HAS_SYSTEM_MODEL:
                self.search_path(next_current_edge)
            else: 
                raise ValueError("todo: define behavior when this happens")
            self.search_hypothesis()

        # move this plotting toward the edge itself
        next_current_edge.set_executing_status()
        self.current_edge = next_current_edge
        self.current_node = self.get_node(self.hypothesis[self.edge_pointer].source)
        if CREATE_SERVER_DASHBOARD:
            self.current_edge.motion_planner.visualise(shortest_path=self.current_edge.path)
            self.visualise()

    def update_system_model(self, ident_edge):
        """ update system model of the next edge. """

        for_edge = self.get_edge(ident_edge.model_for_edge_iden)
        dyn_model = ident_edge.dyn_model
        for_edge.set_has_system_model_status()

        self._setup_drive_controller(for_edge.controller, dyn_model)        

    def go_to_loop(self, loop: str):
        """ go to the searching loop. """

        if loop == EXECUTION_LOOP or loop == SEARCHING_LOOP:

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
        pass

    @abstractmethod
    def create_drive_model(self, controller_name: str):
        """ create a dynamic model for driving. """
        pass
   
    def create_push_controller(self) -> Controller:
        # TODO: not implemented
        possible_controllers = self.get_push_controllers()
        return random.choice(possible_controllers)()

    @abstractmethod
    def get_push_controllers(self) -> list:
        pass

    @abstractmethod
    def create_push_model(self, controller_name: str):
        """ create a dynamic model for pushing. """
        pass

#####################################
### HYPOTHESIS CHECKING FUNCTIONS ###
#####################################
    def hypothesis_completed(self) -> bool:
        """ returns true if the hypothesis is completed, otherwise false. """
        return self.edge_pointer >= len(self.hypothesis)-1

    def is_reachable(self, start_node_iden, target_node_iden) -> bool:
        """ return true if there is a list of edges going from the start node
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
        else:
            return start_node_list[0]
    
    def get_target_node(self, iden) -> Node:
        """ return target node by id, raises error if the identifyer does not exist. """
        target_node_list = [node for node in self.target_nodes if node.iden  == iden]

        if len(target_node_list) == 0:
            raise IndexError(f"a target node with identifyer {iden} does not exist.")
        else:
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
        
        assert isinstance(start_iden, int), f"start_iden should be an int and it {type(start_iden)}"
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
        net.path = "/home/gijs/Documents/semantic-thinking-robot"\
                "/dashboard/assets/graph_template.html"

        net.set_edge_smooth('dynamic')

        for node in self.start_nodes:
            if node == self.current_node:
                continue
            net.add_node(node.iden,
                    title = f"Starting Node: {node.iden}<br>{node.to_string()}<br>",
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
                    title = f"Target Node: {node.iden}<br>{node.to_string()}<br>",
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
                    title = f"Node: {node.iden}<br>{node.to_string()}<br>",
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
                    title = f"Current node: {self.current_node.iden}<br>{self.current_node.to_string()}<br>",
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
                color = '#fb4b50'
            else:
                color = "#438ced"

            dashes = False
            if edge.path is False:
                dashes = True

            net.add_edge(edge.source,
                    edge.to,
                    dashes=dashes,
                    width=value,
                    color=color,
                    label=edge.verb,
                    title="edge:<br>"  + edge.to_string() + "<br>",
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
            print('TOdo, this sanitasation here for Ident edge as curretn edge')
        else: 
            ValueError('error here')
        # more input sanitisation
        self._current_edge = edge

    @property
    def obstacles(self):
        return self._obstacles

    @obstacles.setter
    def obstacles(self, obstacles):
        assert isinstance(obstacles, dict), f"obstacles should be a dictionary and is {type(obstacles)}"
        self._obstacles = obstacles

    @property
    def robot_order(self):
        return self._robot_order

    @robot_order.setter
    def robot_order(self, val):
        assert isinstance(val, int), f"robot_order's type should be an int and is {type(val)}"
        assert val > 0, f"robot order should be higher than 0 and is {val}"
        self._robot_order = val
