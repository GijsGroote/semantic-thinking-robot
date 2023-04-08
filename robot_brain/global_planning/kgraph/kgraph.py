from pyvis.network import Network
from robot_brain.global_planning.graph import Graph
from robot_brain.global_planning.node import Node
from typing import Tuple

from robot_brain.global_variables import FIG_BG_COLOR, COLORS, PROJECT_PATH, LOG_METRICS, CREATE_SERVER_DASHBOARD, SAVE_LOG_METRICS
from robot_brain.global_planning.hgraph.action_edge import ActionEdge, EDGE_PATH_EXISTS, EDGE_PATH_IS_PLANNED, EDGE_HAS_SYSTEM_MODEL
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
        for node in self.nodes.values():
            print(f'node name: {node.obj.name}, iden: {node.iden}, type: {node.obj.type}')
        print(" ")

    def add_object(self, obj: Object):
        """ adds new object to the kgraph. """
        if not isinstance(obj, Object):
            raise TypeError("Object's only")
        assert obj.type in [MOVABLE, UNMOVABLE], f"added object must have type MOVABLE or UNMOVABLE and is {obj.type}"

        # check if the object is not already in the kgraph
        for temp_node in self.nodes.values():
            if temp_node.obj.name == obj.name:
                raise TypeError(f"object with name: {obj.name} is already in the KGraph")

        obj_node = ObjectNode(self.unique_node_iden(), obj.name, obj, "no_subtask_name")
        self.add_node(obj_node)

    def get_object_type(self, obj_name: str) -> int:
        """ return the type of the object if known. """

        for node in self.nodes.values():
            if node.obj.name == obj_name:
                return node.obj.type

        return None

    def add_edge_review(self, edge: ActionEdge):
        """ add a review edge to the KGraph. """
        # TODO: make this function
        pass

    def action_suggestion(self, edge: ActionEdge) -> Tuple[Controller, SystemModel]:
        """ add a review edge to the KGraph. """
        # TODO: make this function
        pass

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
        assert not node.name in self.nodes, f"node.name: {node.name} already exist in self.nodes"
        self.nodes[node.name] = node

    def visualise(self, save=True):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """

        net = Network(bgcolor=FIG_BG_COLOR, height="450px", directed=True)

        # set a custom style sheet
        net.path = PROJECT_PATH+"/dashboard/assets/graph_template.html"

        net.set_edge_smooth('dynamic')

        for node in self.nodes:

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
        # add edges
        for edge in self.edges:

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
