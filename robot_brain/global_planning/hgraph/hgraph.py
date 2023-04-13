""" The Hypothesis Graph (HGraph) is a graph-based structure that represents the
robot environment. Nodes correspond to an object in the environment at a given
configuration, edges represent actions that can give objects in the environment a
new configuration. Implemented actions are nonprehensile pushing and robot driving.
"""

from abc import abstractmethod
from typing import Tuple
from pyvis.network import Network

from robot_brain.object import Object
from robot_brain.state import State

from robot_brain.local_planning.graph_based.path_estimator import PathEstimator
from robot_brain.local_planning.sample_based.push_motion_planner import PushMotionPlanner

from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH

from robot_brain.global_planning.graph import Graph
from robot_brain.global_planning.object_node import ObjectNode
from robot_brain.global_planning.node import Node, NODE_COMPLETED, NODE_UNFEASIBLE, NODE_INITIALISED, NODE_FAILED
from robot_brain.global_planning.edge import Edge, EDGE_INITIALISED, EDGE_COMPLETED, EDGE_FAILED
from robot_brain.global_planning.hgraph.empty_edge import EmptyEdge
from robot_brain.global_planning.hgraph.identification_edge import IdentificationEdge
from robot_brain.global_planning.hgraph.action_edge import ActionEdge, EDGE_PATH_EXISTS
from robot_brain.global_planning.hgraph.drive_act_edge import DriveActionEdge
from robot_brain.global_planning.hgraph.push_act_edge import PushActionEdge

from robot_brain.controller.push.push_controller import PushController
from robot_brain.controller.drive.drive_controller import DriveController
from robot_brain.controller.controller import Controller
from robot_brain.system_model import SystemModel

from robot_brain.exceptions import (
        RunnoutOfControlMethodsException,
        LoopDetectedException,
        TwoEdgesPointToSameNodeException,
        )

ROBOT_IDEN = 0

class HGraph(Graph):
    """
    Hypothesis graph.
    """
    def __init__(self, robot_obj: Object, robot_order: int):

        Graph.__init__(self)

        # create and add node containing the robot itself
        self.robot_obj = robot_obj
        self.robot_order = robot_order

        self.start_nodes = {}
        self.target_nodes = {}
        self.c_node = None
        self.c_edge = None

        # blocklist containing banned edges
        self.blocklist = {}

    ##########################################
    ### adding/editing edges and nodes #######
    ##########################################
    def add_node(self, node: ObjectNode):
        """ add node to the dictionary of nodes. """
        assert isinstance(node, ObjectNode), f"node should be of type ObjectNode and is {type(node)}"
        assert isinstance(node.obj, Object), f"node.obj should of type Object and is {type(node.obj)}"
        assert not node.iden in self.nodes, f"node.iden: {node.iden}, is already present in self.nodes"

        self.nodes[node.iden] = node
        assert self.is_valid_check()

    def add_start_node(self, node: ObjectNode):
        """ add starting node. """
        assert not node.iden in self.start_nodes, f"node.iden: {node.iden}, is already present in self.start_nodes"

        self.add_node(node)
        self.start_nodes[node.iden] = node

    def get_start_node(self, iden: int) -> Node:
        """ return start node by identifier. """
        assert isinstance(iden, int), f"iden should be of type int and is {type(iden)}"
        for temp_start_node in self.start_nodes:
            if temp_start_node.iden == iden:
                return temp_start_node
        raise ValueError(f"start node with iden {iden} does not exist")

    def add_target_node(self, node: ObjectNode):
        """ add target node. """
        assert not node.iden in self.target_nodes, f"node.iden: {node.iden}, is already present in self.target_nodes"
        self.add_node(node)
        self.target_nodes[node.iden] = node

    def get_start_node_from_target_node(self, target_node: ObjectNode) -> ObjectNode:
        """ return start node from subtask with target_node_iden. """
        assert isinstance(target_node, ObjectNode)
        for temp_node in self.start_nodes.values():
            if temp_node.subtask_name == target_node.subtask_name and temp_node.obj.properties == target_node.obj.properties:
                return temp_node
        raise ValueError(f"corresponding start node not found from target node with iden: {target_node.iden}")

    def get_target_node_from_start_node_iden(self, iden: int) -> Node:
        """ return target node that corresponds to start node identifier. """
        assert isinstance(iden, int), f"iden should be of type int and is {type(iden)}"
        assert iden in self.start_nodes, "iden is not in in start_nodes"

        for temp_target_node in self.target_nodes.values():
            if temp_target_node.obj.properties == self.start_nodes[iden].obj.properties:
                return temp_target_node
        raise ValueError("target node could not be found from starting node iden")

    def get_robot_start_node_in_subtask(self, subtask_name: str) -> ObjectNode:
        """ return the robot start node that is in subtask with subtask_name. """
        assert isinstance(subtask_name, str)
        for temp_start_node in self.start_nodes.values():
            if temp_start_node.obj.properties == self.robot_obj.properties and temp_start_node.subtask_name == subtask_name:
                return temp_start_node
        raise ValueError("robot start node could not be found")


    def get_connected_source_node(self, target_node: ObjectNode, subtask_name: str) -> ObjectNode:
        """ find the node that points to the target_node over non-failing nodes and edges all in the same subtask. """
        assert isinstance(target_node, ObjectNode)
        assert isinstance(subtask_name, str)

        incoming_edge_list = [edge for edge in self.edges.values() if\
                edge.to == target_node.iden and\
                edge.status != EDGE_FAILED and\
                self.get_node(edge.source).status == NODE_INITIALISED and\
                edge.subtask_name == subtask_name]

        # no valid edges pointing toward this node -> return
        if len(incoming_edge_list) == 0 or\
                self.get_node(incoming_edge_list[0].source).status in [NODE_UNFEASIBLE, NODE_UNFEASIBLE]:
            return target_node

        else:
            return self.get_connected_source_node(self.get_node(incoming_edge_list[0].source), subtask_name)

    def get_connected_target_node(self, source_node: ObjectNode, subtask_name: str) -> ObjectNode:
        """ find the node that points to the target_node over non-failing nodes and edges all in the same subtask. """
        assert isinstance(source_node, ObjectNode)
        assert isinstance(subtask_name, str)

        outgoing_edge_list = [edge for edge in self.edges.values() if\
                edge.source == source_node.iden and\
                edge.status != EDGE_FAILED and\
                self.get_node(edge.to).status in [NODE_INITIALISED, NODE_COMPLETED] and\
                edge.subtask_name == subtask_name]

        # no valid edges pointing out of this node -> return
        if len(outgoing_edge_list) == 0 or\
                self.get_node(outgoing_edge_list[0].to).status in [NODE_UNFEASIBLE, NODE_UNFEASIBLE]:
            return source_node

        else:
            return self.get_connected_target_node(self.get_node(outgoing_edge_list[0].to), subtask_name)

    def fail_edge(self, edge: Edge):
        """
        fail edge and incoming/outgoing emtpy edges
        or an action edge fail corresponding identification edge.
        all failed edges are in the same subtask
        """

        edge.status = EDGE_FAILED

        # fail incoming/outgoing emtpy edges
        for temp_incoming_edge in self.get_incoming_edges(edge.source):
            if isinstance(temp_incoming_edge, EmptyEdge) and temp_incoming_edge.subtask_name == edge.subtask_name:
                temp_incoming_edge.status = EDGE_FAILED

        for temp_outgoing_edge in self.get_outgoing_edges(edge.to):
            if isinstance(temp_outgoing_edge, EmptyEdge) and temp_outgoing_edge.subtask_name == edge.subtask_name:
                temp_outgoing_edge.status = EDGE_FAILED

        if isinstance(edge, ActionEdge):
            # fail corresponding identification edge
            for temp_edge in self.edges.values():
                if isinstance(temp_edge, IdentificationEdge) and temp_edge.model_for_edge_iden == edge.iden:
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

    def filter_control_and_model_names(self, edge_class: str, control_and_models: list, target_iden: int) -> list:
        """ removes the controllers and edges that are on the blocklist from control_and_models. """
        assert edge_class in [DriveActionEdge, PushActionEdge]
                # f"edge should be an DriveActionEdge or PushActionEdge and is {type(edge_class)}"
        assert isinstance(control_and_models, list), f"control_and_models should be an list and is {type(control_and_models)}"
        for (temp_controller, temp_model_name_list) in control_and_models:
            assert isinstance(temp_controller, Controller)
            assert isinstance(temp_model_name_list, list)
            for temp_model_name in temp_model_name_list:
                assert isinstance(temp_model_name, str)
        assert isinstance(target_iden, int), f"target_iden should be an int and is {type(target_iden)}"

        controller_and_model_names_filtered = []
        # filter out blocked combinations of control methods with system models

        if target_iden in self.blocklist:
            for (controller, model_names) in control_and_models:

                model_names_filtered = []
                for model_name in model_names:

                    if not self.in_blocklist(
                        target_iden,
                        self.get_node(target_iden).obj.name,
                        str(type(controller)),
                        model_name):
                        model_names_filtered.append(model_name)

                if len(model_names_filtered) > 0:
                    controller_and_model_names_filtered.append((controller, model_names_filtered))
        else:
            controller_and_model_names_filtered = control_and_models

        if len(controller_and_model_names_filtered) == 0:
            raise RunnoutOfControlMethodsException("All possible edges are on the blocklist")

        return controller_and_model_names_filtered

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
    def in_object(self, pose_2ds: list, obj: Object, objects: dict) -> list:
        """ return the object keys from objects at pose_2ds that are in collision with obj. """

    @abstractmethod
    def find_best_push_state_againts_object(self, blocking_obj: Object, path: list) -> State:
        """ return a state to push against blocking_obj to later push blocking_obj over path. """

    @abstractmethod
    def find_free_state_for_blocking_object(self, blocking_obj: Object, path: list) -> State:
        """ return a state where the object can be pushed toward so it is not blocking the path. """

    ##########################################
    ### checking / validating  ###############
    ##########################################

    def is_reachable(self, source_node: ObjectNode, target_node: ObjectNode) -> bool:
        """ return true if there is a list of non-failed edges going from the start node
        to target node all in the same subtask, otherwise return false. """
        assert isinstance(source_node, ObjectNode), f"source_node should be an ObjectNode and is {type(source_node)}"
        assert isinstance(target_node, ObjectNode), f"target_node should be an ObjectNode and is {type(target_node)}"
        assert source_node.subtask_name == target_node.subtask_name,\
            f"source subtask_name: {source_node.subtask_name} is not equal to target subtask_name: {target_node.subtask_name}"

        reachable_from_start = [source_node.iden]

        while len(reachable_from_start) > 0:
            temp_node_iden = reachable_from_start.pop(0)

            for outgoing_edge in self.get_outgoing_edges(temp_node_iden):
                if outgoing_edge.to == target_node.iden:
                    return True
                elif outgoing_edge.subtask_name == source_node.subtask_name and\
                        outgoing_edge.status != EDGE_FAILED:
                    reachable_from_start.append(outgoing_edge.to)

        return False

    def is_valid_check(self) -> bool:
        """
        for edges and nodes in the same subtask:
            check if there are no loops
            check if there are not multiple non-failing edges pointing toward the same node
        """
        for temp_node in self.nodes.values():
            node_points_to_temp_node = temp_node

            while node_points_to_temp_node:

                # find edges pointing toward temp_node
                edge_points_to_temp_node_list = [edge for edge in self.edges.values() if\
                        edge.to == node_points_to_temp_node.iden and\
                        edge.status != EDGE_FAILED and\
                        self.get_node(edge.to).subtask_name == temp_node.subtask_name and\
                        self.get_node(edge.source).subtask_name == temp_node.subtask_name]

                if len(edge_points_to_temp_node_list) == 0:
                    break
                if len(edge_points_to_temp_node_list) > 1:
                    self.visualise(save=False)
                    raise TwoEdgesPointToSameNodeException()

                node_points_to_temp_node = self.get_node(edge_points_to_temp_node_list[0].source)

                if node_points_to_temp_node.iden == temp_node.iden:

                    self.visualise(save=False)
                    raise LoopDetectedException

        return True

    def in_blocklist(self,
            target_iden: int,
            obj_name: str,
            controller_type: str,
            system_model_name: str) -> bool:
        """ checks if the edge is already in the blocklist. """
        assert isinstance(target_iden, int)
        assert isinstance(obj_name, str)
        assert isinstance(controller_type, str)
        assert isinstance(system_model_name, str)

        if target_iden in self.blocklist:
            for blocked_dict in self.blocklist[target_iden]:

                if target_iden == blocked_dict["target_iden"] and\
                    obj_name == blocked_dict["obj_name"] and\
                    controller_type == blocked_dict["controller_type"] and\
                    system_model_name == blocked_dict["system_model_name"]:
                    return True

        return False

    def add_to_blocklist(self, edge: ActionEdge):
        """ add edge to the blocklist. """
        assert isinstance(edge, ActionEdge), f"edge should be ActionEdge and is {type(edge)}"
        assert edge.controller.system_model.name is not None,\
                "cannot add edge without system model name"
        assert callable(edge.controller.system_model.model),\
                "cannot add edge without callable system model"

        edge_blocked_dict = {
                "target_iden": edge.to,
                "obj_name": self.get_node(edge.to).obj.name,
                "controller_type": str(type(edge.controller)),
                "system_model_name": edge.controller.system_model.name
                }

        assert not self.in_blocklist(
                edge_blocked_dict["target_iden"],
                edge_blocked_dict["obj_name"],
                edge_blocked_dict["controller_type"],
                edge_blocked_dict["system_model_name"]),\
                f"edge: {edge.iden} should not already be in blocklist"

        # create blocked dictionary on target node key
        if edge.to in self.blocklist:
            self.blocklist[edge.to].append(edge_blocked_dict)
        else:
            self.blocklist[edge.to] = [edge_blocked_dict]

    ##########################################
    ### setters and getters ##################
    ##########################################

    #TODO: all setters and getters from INIT
    @property
    def robot_order(self):
        return self._robot_order

    @robot_order.setter
    def robot_order(self, val):
        assert isinstance(val, int),\
                f"robot_order's type should be an int and is {type(val)}"
        assert val > 0, f"robot order should be higher than 0 and is {val}"
        self._robot_order = val

    @property
    def c_node(self):
        return self._c_node

    @c_node.setter
    def c_node(self, node) -> ObjectNode:
        assert isinstance(node, (Node, type(None))), f"node should be a Node or None and is {type(node)}"
        if isinstance(node, Node):
            assert node.iden in self.nodes, "node should be in self.nodes"
        self._c_node = node

    @property
    def c_edge(self) -> Edge:
        return self._c_edge

    @c_edge.setter
    def c_edge(self, edge: Edge):
        assert isinstance(edge, (Edge, type(None))), f"edge should be an Edge or None and is {type(edge)}"
        if isinstance(edge, Edge):
            self.c_node = self.get_node(edge.source)
        self._c_edge = edge


    ##########################################
    ### visualise functionality ##############
    ##########################################
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

        for node in self.start_nodes.values():
            if node.name == "pointRobot-vel-v7": # deltete this
                node.name = "robot"

            if node == self.c_node:
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

        for node in self.target_nodes.values():

            if node.name == "pointRobot-vel-v7_target":
                node.name = "robot_target"

            if node == self.c_node:
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

        for node in self.nodes.values():
            if node == self.c_node:
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

        if self.c_node is not None:
            net.add_node(self.c_node.iden,
                    title = f"Current Node: {self.c_node.name}<br>{self.c_node.to_string()}<br>",
                    x=10.0,
                    y=10.0,
                    label = self.c_node.name,
                    color= {
                        'border': '#fb4b50',
                        'background': '#fb7e81',
                        'highlight': {
                            'border': '#fb4b50',
                            'background': '#fcbcc4'
                            }
                        },
                    group = "c_node")

        # add edges
        for edge in self.edges.values():

            value = 1.5
            # if edge in hypothesis:
            #     value = 3

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
