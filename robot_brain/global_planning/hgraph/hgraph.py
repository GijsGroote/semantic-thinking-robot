""" The Hypothesis Graph (HGraph) is a graph-based structure that represents the
robot environment. Nodes correspond to an object in the environment at a given
configuration, edges represent actions that can give objects in the environment a
new configuration. Implemented actions are nonprehensile pushing and robot driving.
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
from robot_brain.controller.push.push_controller import PushController
from robot_brain.local_planning.sample_based.push_motion_planner import PushMotionPlanner
from robot_brain.local_planning.sample_based.drive_motion_planner import DriveMotionPlanner 
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

ROBOT_IDEN = 0

class HGraph(Graph):
    """
    Hypothesis graph.
    """
    def __init__(self, robot_obj: Object):
        Graph.__init__(self)

        # node containing the robot itself
        self.robot_node = ObjectNode(ROBOT_IDEN, robot_obj.name, robot_obj)

        self.start_nodes = []
        self.target_nodes = []
        self.current_node = None
        self.current_edge = None

        # blocklist containing banned edges
        self.blocklist = {}
        print("Hyp Graph created")

    ##########################################
    ### adding/editing edges and nodes #######
    ##########################################
    def add_node(self, node: ObjectNode):
        """ add regular node that is not a start or target node. """
        assert isinstance(node, ObjectNode), f"node should be of type ObjectNode and is {type(node)}"
        assert isinstance(node.object, Object), f"node.object should of type Object and is {type(node.object)}"
        self.nodes.append(node)

    def add_start_node(self, node: ObjectNode):
        """ add starting node. """
        self.add_node(node)
        self.start_nodes.append(node)

    def get_start_node(self, iden: int) -> Node:
        """ return start node by identifier. """
        assert isinstance(iden, int), f"iden should be of type int and is {type(iden)}"
        for temp_start_node in self.start_nodes:
            if temp_start_node.iden == iden:
                return temp_start_node
        raise ValueError(f"start node with iden {iden} does not exist")

    def add_target_node(self, node: ObjectNode):
        """ add target node. """
        if isinstance(node, ChangeOfStateNode):
            raise TypeError("ChangeOfStateNode's are not allowed as target node in HGraph")
        self.add_node(node)
        self.target_nodes.append(node)

    def get_target_node(self, iden: int) -> Node:
        """ return target node by identifier. """
        assert isinstance(iden, int), f"iden should be of type int and is {type(iden)}"

        for temp_target_node in self.start_nodes:
            if temp_target_node.iden == iden:
                return temp_target_node
        raise ValueError(f"target node with iden {iden} does not exist")


    def fail_edge(self, edge: Edge):
        """
        fail edge and incoming/outgoing emtpy edges
        for an action edge fail corresponding identification edge.
        """

        edge.status = EDGE_FAILED

        # fail incoming/outgoing emtpy edges
        if isinstance(self.get_incoming_edge(edge.source), EmptyEdge):
            self.get_incoming_edge(edge.source).status = EDGE_FAILED
        for outgoing_edge in self.get_outgoing_edges(edge.to):
            if isinstance(outgoing_edge, EmptyEdge):
                outgoing_edge.status = EDGE_FAILED

        if isinstance(edge, ActionEdge):
            # fail corresponding identification edge
            for temp_edge in self.edges:
                if temp_edge.model_for_edge_iden == edge.iden and isinstance(temp_edge, IdentificationEdge):
                    self.fail_edge(temp_edge)

    ##########################################
    ### path estimation and planning #########
    ##########################################

    @abstractmethod
    def create_drive_path_estimator(self, objects: dict) -> PathEstimator:
        """ create drive path estimator. """

    @abstractmethod
    def create_push_path_estimator(self, push_obj: Object, objects: dict) -> PathEstimator:
        """ create push path estimator. """

    @abstractmethod
    def create_drive_motion_planner(self, objects: dict, path_estimator: PathEstimator):
        """ create drive motion planner. """

    @abstractmethod
    def create_push_motion_planner(self, objects: dict, push_obj: Object, path_estimator: PathEstimator) -> PushMotionPlanner:
        """ create push manipulation planner. """

    ##########################################
    ### creating controllers #################
    ##########################################
    @abstractmethod
    def create_drive_controller(self, target_iden: int) -> Tuple[Controller, str]:
        """ randomly select a driving controller that is not on the blocklist. """

    @abstractmethod
    def create_drive_model(self, model_name: str) -> SystemModel:
        """ create the requested drive system model. """

    @abstractmethod
    def setup_drive_controller(self, controller: DriveController, system_model: SystemModel):
        """ setup drive controller """

    @abstractmethod
    def create_push_controller(self, target_iden: int) -> Tuple[Controller, str]:
        """ create push controller. """

    @abstractmethod
    def create_push_model(self, model_name: str) -> SystemModel:
        """ create the requested push system model. """

    @abstractmethod
    def setup_push_controller(self, controller: PushController, system_model: SystemModel, push_edge: PushActionEdge):
        """ setup push controller """

    def update_system_model(self, ident_edge: IdentificationEdge):
        """ update system model of the corresponding edge. """
        assert isinstance(ident_edge, IdentificationEdge), f"ident_edge should be IdentificationEdge and is {type(ident_edge)}"

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
    def find_compatible_models(self, controllers: list) -> list:
        """ return compatible system models for controllers. """

    ##########################################
    ### finding specific configurations ######
    ##########################################
    @abstractmethod
    def in_object(self, pose_2ds: list, obj: Object) -> list:
        """ return the object keys at pose_2ds that are in collision with obj. """

    @abstractmethod
    def find_best_push_state_againts_object(self, blocking_obj: Object, path: list) -> State:
        """ return a state to push against blocking_obj to later push blocking_obj over path. """

    @abstractmethod
    def find_free_state_for_blocking_object(self, blocking_obj: Object, path: list) -> State:
        """ return a state where the object can be pushed toward so it is not blocking the path. """

    ##########################################
    ### checking / validating  ###############
    ##########################################

    def is_reachable(self, source_node_iden: int, target_node_iden: int) -> bool:
        """ return true if there is a list of non-failed edges going from the start node
        identifier to target node identifier, otherwise return false. """
        assert isinstance(source_node_iden, int), f"source_node_iden should be an int and is {type(source_node_iden)}"
        assert isinstance(target_node_iden, int), f"target_node_iden should be an int and is {type(target_node_iden)}"

        reachable_from_start = [source_node_iden]

        while len(reachable_from_start) > 0:
            current_node_iden = reachable_from_start.pop(0)

            for outgoing_node_iden in self.point_toward_nodes(current_node_iden):
                if outgoing_node_iden == target_node_iden:
                    return True
                reachable_from_start.append(outgoing_node_iden)

        return False

    def in_blocklist(self, para_edge_blocked: tuple) -> bool:
        """ checks if the edge is already in the blocklist. """
        assert isinstance(para_edge_blocked, tuple), f"edge_blocked should be list and is {type(edge_blocked)}"
        assert isinstance(para_edge_blocked[0], int)
        assert isinstance(para_edge_blocked[1], str)
        assert isinstance(para_edge_blocked[2], Edge)
        assert isinstance(para_edge_blocked[3], str)
        assert isinstance(para_edge_blocked[4], str)

        if para_edge_blocked[0] in self.blocklist:
            for temp_para_edge in self.blocklist[para_edge_blocked[0]]:
                tpe = temp_para_edge

                if para_edge_blocked[0] == tpe[0] and\
                    para_edge_blocked[1] == tpe[1] and\
                    para_edge_blocked[2] == tpe[2] and\
                    para_edge_blocked[3] == tpe[3] and\
                    para_edge_blocked[4] == tpe[4]:
                    return True

        return False

    def add_to_blocklist(self, edge: ActionEdge):
        """ add edge to the blocklist. """
        assert isinstance(edge, ActionEdge), f"edge should be ActionEdge and is {type(edge)}"
        assert edge.controller.system_model.name is not None,\
                "cannot add edge without system model name"
        assert callable(edge.controller.system_model.model),\
                "cannot add edge without callable system model"

        edge_blocked = (
                edge.to,
                self.get_node(edge.to).obj.name,
                type(edge),
                edge.controller.name,
                edge.controller.system_model.name)

        assert not self.in_blocklist(edge_blocked),\
                f"edge: {edge.iden} should not already be in blocklist"

        # create blocklist on target node key
        if edge.to in self.blocklist:
            self.blocklist[edge.to].append(edge_blocked)
        else:
            self.blocklist[edge.to] = [edge_blocked]

    ##########################################
    ### setters and getters ##################
    ##########################################
    @property
    def current_node(self):
        return self._current_node

    @current_node.setter
    def current_node(self, node) -> ObjectNode:
        assert isinstance(node, Node), f"node should be a Node and is {type(node)}"
        assert node in self.nodes, "node should be in self.nodes"
        assert node.ready_for_execution(),\
                f"node.ready_for_execution() should be True and is: {node.ready_for_execution()}"
        self._current_node = node

    @property
    def current_edge(self) -> Edge:
        return self._current_edge

    @current_edge.setter
    def current_edge(self, edge: Edge):
        assert isinstance(edge, Edge), f"edge should be an Edge and is {type(edge)}"
        assert edge.ready_for_execution(),\
                f"edge.ready_for_execution() should be True and is: {edge.ready_for_execution()}"

        self._current_edge = edge

    def visualise(self, hypothesis=[], save=True):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """
        assert isinstance(hypothesis, list), f"hypothesis should be list and is {type(hypothesis)}"
        for temp_edge in hypothesis:
            assert isinstance(temp_edge, Edge), f"hypothesis should contain only Edge and contains a: {type(temp_edge)}"
        assert isinstance(save, bool), f"save should be bool and is {type(save)}"

        net = Network(bgcolor=FIG_BG_COLOR, height="450px", directed=True)

        # set a custom style sheet
        net.path = PROJECT_PATH+"/dashboard/assets/graph_template.html"

        net.set_edge_smooth('dynamic')

        for node in self.start_nodes:
            if node.name == "pointRobot-vel-v7": # deltete this
                node.name = "robot"

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

            if node.name == "pointRobot-vel-v7_target":
                node.name = "robot_target"

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

            if node.name == "pointRobot-vel-v7_model":
                node.name = "robot_model"

            if node.name == "pointRobot-vel-v7_copy":
                node.name = "robot_copy"

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
                    group = "current_node")

        # add edges
        for edge in self.edges:

            value = 1.5
            if edge in hypothesis:
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


