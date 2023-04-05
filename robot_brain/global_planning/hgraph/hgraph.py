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

        # blacklist containing banned edges
        self.blacklist = {}
        print("Hyp Graph created")

    ##########################################
    ### adding/editing edges and nodes #######
    ##########################################
    def add_node(self, node):
        """ add regular node that is not a start or target node. """
        assert isinstance(node, ObjectNode), f"node should be of type ObjectNode and is {type(node)}"
        assert isinstance(node.object, Object), f"node.object should of type Object and is {type(node.object)}"
        self.nodes.append(node)

    def add_start_node(self, node):
        """ add starting node. """
        self.add_node(node)
        self.start_nodes.append(node)

    def get_start_node(self, iden) -> Node:
        """ return start node by identifier. """
        assert isinstance(iden, int), f"iden should be of type int and is {type(iden)}"
        for temp_start_node in self.start_nodes:
            if temp_start_node.iden == iden:
                return temp_start_node
        raise ValueError(f"start node with iden {iden} does not exist")

    def add_target_node(self, node):
        """ add target node. """
        if isinstance(node, ChangeOfStateNode):
            raise TypeError("ChangeOfStateNode's are not allowed as target node in HGraph")
        self.add_node(node)
        self.target_nodes.append(node)

    def get_target_node(self, iden) -> Node:
        """ return target node by identifier. """
        assert isinstance(iden, int), f"iden should be of type int and is {type(iden)}"

        for temp_target_node in self.start_nodes:
            if temp_target_node.iden == iden:
                return temp_target_node
        raise ValueError(f"target node with iden {iden} does not exist")


    def get_start_iden_from_target_iden(self, target_iden):
        """ returns the starting node identifier from a target node identifier. """

    def fail_edge(self, edge: Edge):
        """ fail edge and corresponding identification and empty edges. """

    ##########################################
    ### path estimation and planning #########
    ##########################################

    @abstractmethod
    def create_drive_path_estimator(self, objects) -> PathEstimator:
        """ create drive path estimator. """

    @abstractmethod
    def create_push_path_estimator(self, push_obj, objects) -> PathEstimator:
        """ create push path estimator. """

    def estimate_path(self, edge):
        """ estimate path existance for edge. """

    @abstractmethod
    def create_drive_motion_planner(self, objects, path_estimator):
        """ create drive motion planner. """

    @abstractmethod
    def create_push_motion_planner(self, objects, push_obj, path_estimator):
        """ create push manipulation planner. """

    def search_path(self, edge):
        """ search for a path from start to target for an edge. """

    ##########################################
    ### creating controllers #################
    ##########################################
    @abstractmethod
    def create_drive_controller(self, target_iden: int) -> Tuple[Controller, str]:
        """ randomly select a driving controller that is not on the blacklist. """

    @abstractmethod
    def create_drive_model(self, model_name: str) -> SystemModel:
        """ create the requested drive system model. """

    @abstractmethod
    def setup_drive_controller(self, controller, system_model):
        """ setup drive controller """

    @abstractmethod
    def create_push_controller(self, target_iden: int) -> Tuple[Controller, str]:
        """ create push controller. """

    @abstractmethod
    def create_push_model(self, model_name: str):
        """ create the requested push system model. """

    @abstractmethod
    def setup_push_controller(self, controller, system_model, push_edge):
        """ setup push controller """

    def update_system_model(self, ident_edge):
        """ update system model of the corresponding edge. """

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
    def find_best_push_state_againts_object(self, blocking_obj, path) -> State:
        """ return a state to push against blocking_obj to later push blocking_obj over path. """

    @abstractmethod
    def find_free_state_for_blocking_object(self, blocking_obj: Object, path: list) -> State:
        """ return a state where the object can be pushed toward so it is not blocking the path. """

    ##########################################
    ### checking / validating  ###############
    ##########################################
    def hypothesis_completed(self) -> bool:
        """ returns true if the hypothesis is completed, otherwise false. """

    def is_current_subtask_connected(self) -> bool:
        """ check if the current subtask has a path from
        source node -> target node via non-failed edges. """

    def is_reachable(self, source_node_iden, target_node_iden) -> bool:
        """ return true if there is a list of non-failed edges going from the start node
        identifier to target node identifier, otherwise return false. """

    def in_blacklist(self, edge_type: list) -> bool:
        """ checks if the edge is already in the blacklist. """

    def add_to_blacklist(self, edge):
        """ add edge to the blacklist. """

    ##########################################
    ### setters and getters ##################
    ##########################################

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

    def visualise(self, save=True):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """
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


