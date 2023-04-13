import numpy as np
from pyvis.network import Network
from robot_brain.global_planning.graph import Graph
from robot_brain.global_planning.node import Node
from typing import Tuple

from robot_brain.global_variables import FIG_BG_COLOR, COLORS, PROJECT_PATH, LOG_METRICS, CREATE_SERVER_DASHBOARD, SAVE_LOG_METRICS
from robot_brain.global_planning.hgraph.action_edge import (
        ActionEdge,
        EDGE_PATH_EXISTS,
        EDGE_PATH_IS_PLANNED,
        EDGE_COMPLETED,
        EDGE_HAS_SYSTEM_MODEL,
        EDGE_FAILED)
from robot_brain.system_model import SystemModel
from robot_brain.object import Object, FREE, MOVABLE, UNKNOWN, UNMOVABLE
from robot_brain.controller.controller import Controller

from robot_brain.global_planning.object_node import ObjectNode
from robot_brain.global_planning.kgraph.empty_node import EmptyNode
from robot_brain.global_planning.kgraph.feedback_edge import FeedbackEdge

class KGraph(Graph):
    """
    Knowledge graph.
    """
    def __init__(self):
        Graph.__init__(self)

    def print_kgraph_info(self):
        """ print info of the kgraph. """
        print(f"info on kgraph nodes, n_nodes= {len(self.nodes)}")
        for temp_node in self.nodes.values():
            if isinstance(temp_node, ObjectNode):
                print(f'node name: {temp_node.obj.name}, iden: {temp_node.iden}, type: {temp_node.obj.type}')
        print(" ")

    def add_object(self, obj: Object):
        """ adds new object to the kgraph. """
        if not isinstance(obj, Object):
            raise TypeError("Object's only")
        assert obj.type in [MOVABLE, UNMOVABLE], f"added object must have type MOVABLE or UNMOVABLE and is {obj.type}"
        (obj_node_in_kgraph, _) = self.object_in_kgraph(obj)
        assert not obj_node_in_kgraph, f" object {obj.name} already in kgraph"

        obj_node = ObjectNode(self.unique_node_iden(), obj.name, obj, "no_subtask_name")
        self.add_node(obj_node)

        if CREATE_SERVER_DASHBOARD:
            self.visualise()

    def get_object_type(self, obj: Object) -> int:
        """ return the type of the object if known, otherwise return None. """

        (obj_node_in_kgraph, obj_node) = self.object_in_kgraph(obj)
        if obj_node_in_kgraph:
            return obj_node.type
        else:
            return None

    def add_edge_review(self, obj: Object, edge: ActionEdge):
        """ add a review edge to the KGraph. """
        assert isinstance(obj, Object)
        assert isinstance(edge, ActionEdge)
        assert self.ready_for_edge_review(edge)

        # update review if it already exists
        (edge_review_in_kgraph, feedback_edge) = self.edge_review_in_kgraph(obj, edge)
        if edge_review_in_kgraph:
            self.update_edge_review(feedback_edge, obj, edge)
        else:
            # add object node if it not already exists
            (obj_node_in_kgraph, obj_node) = self.object_in_kgraph(obj)

            if not obj_node_in_kgraph:
                assert obj.type == MOVABLE
                self.add_object(obj)

            # find source node that contains obj in kgraph
            (_, obj_node) = self.object_in_kgraph(obj)

            target_node = EmptyNode(self.unique_node_iden(), obj.name)
            self.add_node(target_node)

            # add new feedbackedge
            self.add_edge(FeedbackEdge(
                iden=self.unique_edge_iden(),
                source=obj_node.iden,
                to=target_node.iden,
                success_factor=self.calculate_successfactor(edge),
                obj=obj,
                verb=edge.verb,
                controller=edge.controller,
                model_name=edge.model_name,
                edge_status=edge.status))


        if CREATE_SERVER_DASHBOARD:
            self.visualise()

    def update_edge_review(self, feedback_edge: FeedbackEdge, obj: ObjectNode, edge: ActionEdge):
        """ update the existing edge review with new feedback. """

        if edge.status == EDGE_COMPLETED:
            feedback_edge.n_success += 1
        elif edge.status == EDGE_FAILED:
            feedback_edge.n_failed += 1
        else:
            raise ValueError(f"incorrect edge_status encountered: {edge.status}")

        b =1-(feedback_edge.n_success/(feedback_edge.n_success + feedback_edge.n_failed))
        feedback_edge.success_factor = self.calculate_successfactor(edge)**max(0.01, b)

        if CREATE_SERVER_DASHBOARD:
            self.visualise()

    def edge_review_in_kgraph(self, obj: Object, edge: ActionEdge) -> Tuple[bool, FeedbackEdge]:
        """
        return true if there exists an edge
        the obj_name, controller_name and system_model_name
        equal to the ones obj and edge.
        """
        for temp_node in self.nodes.values():
            if isinstance(temp_node, ObjectNode) and temp_node.obj.name == obj.name:
                for temp_edge in self.get_outgoing_edges(temp_node.iden):
                    if (isinstance(temp_edge.controller, type(edge.controller)) and\
                            temp_edge.model_name == edge.model_name):
                        return (True, temp_edge)

        return (False, None)

    def object_in_kgraph(self, obj: Object) -> Tuple[bool, ObjectNode]:
        """ return true if there exist a node containing the object. """
        for temp_node in self.nodes.values():
            if isinstance(temp_node, ObjectNode) and temp_node.obj.name == obj.name:
                return (True, temp_node)
        return (False, None)

    def ready_for_edge_review(self, edge: ActionEdge) -> bool:
        """ check if edge has all componentes to create a success factor. """
        return len(edge.controller.pred_error) > 0 and edge.status in [EDGE_COMPLETED, EDGE_FAILED]

    def calculate_successfactor(self, edge: ActionEdge) -> float:
        """ calculate a successfactor. """

        return float(max(0.5-np.abs(np.average(edge.controller.pred_error)), 0.1))

    def action_suggestion(self, obj: Object) -> Controller:
        """ query KGraph for action suggestion. """
        assert isinstance(obj, Object)

        (obj_node_in_kgraph, obj_node) = self.object_in_kgraph(obj)
        if not obj_node_in_kgraph:
            return None

        highest_success_factor = 0
        hightest_edge = None
        for temp_edge in self.get_outgoing_edges(obj_node.iden):
            if temp_edge.success_factor > highest_success_factor:
                highest_success_factor = temp_edge.success_factor
                hightest_edge = temp_edge

        if hightest_edge is None:
            return None
        else:
            return hightest_edge.controller

    def all_action_suggestions(self, obj: Object) -> Controller:
        """ query KGraph for action suggestion. """
        assert isinstance(obj, Object)

        (obj_node_in_kgraph, obj_node) = self.object_in_kgraph(obj)
        if not obj_node_in_kgraph:
            return None

        all_action_suggestions = []
        for temp_edge in self.get_outgoing_edges(obj_node.iden):
            all_action_suggestions.append(temp_edge.controller)

        return all_action_suggestions

    def is_valid_check(self) -> bool:
        """
        for edges and nodes in the same subtask:
            check if there are no loops
            check if there are not multiple non-failing edges pointing toward the same node
        """
        #TODO: this methods
        return True

    def add_node(self, node: ObjectNode):
        """ add node to the dictionary of nodes. """
        assert not node.iden in self.nodes, f"node.name: {node.name} already exist in self.nodes"
        self.nodes[node.iden] = node

        if CREATE_SERVER_DASHBOARD:
            self.visualise()

    def visualise(self, save=True):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """

        net = Network(bgcolor=FIG_BG_COLOR, height="450px", directed=True)

        # set a custom style sheet
        net.path = PROJECT_PATH+"/dashboard/assets/graph_template.html"

        net.set_edge_smooth('dynamic')

        for node in self.nodes.values():

            if isinstance(node, ObjectNode):

                net.add_node(node.iden,
                        title = f"Node: {node.name}<br>{node.to_string()}<br>",
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
                        group = "nodes")

            elif isinstance(node, EmptyNode):

                net.add_node(node.iden,
                        x=10.0,
                        y=10.0,
                        label=" ",
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
                        group = "nodes")
            else:
                raise ValueError("unknown node type encountered")
        # add edges
        for edge in self.edges.values():

            value = 1.5

            net.add_edge(edge.source,
                    edge.to,
                    # dashes=dashes,
                    width=value,
                    # color=color,
                    label=edge.verb,
                    title=f"{edge.to_string()}<br>",
                    )

        # if you want to edit cusomize the graph
        # net.show_buttons(filter_=['physics'])

        if save:
            net.write_html(name=PROJECT_PATH+"dashboard/data/knowledge_graph.html")
        else:
            net.show("delete2.html")
