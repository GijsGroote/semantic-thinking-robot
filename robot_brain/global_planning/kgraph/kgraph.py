import os
from pyvis.network import Network
from robot_brain.global_planning.graph import Graph
from robot_brain.global_planning.node import Node

from robot_brain.global_variables import FIG_BG_COLOR, COLORS, PROJECT_PATH, LOG_METRICS, CREATE_SERVER_DASHBOARD, SAVE_LOG_METRICS
from robot_brain.obstacle import Obstacle, FREE, MOVABLE, UNKNOWN, UNMOVABLE
from robot_brain.global_planning.obstacle_node import ObstacleNode
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode


class KGraph(Graph):
    """
    Knowledge graph.
    """
    def __init__(self):
        print('CEATE KGRAPH ONLYL ONCE, SHOULD BE ONECE ONCE CONCE')
        Graph.__init__(self)

    def add_object(self, obj: Obstacle):
        """ adds new obstacle to the kgraph. """
        if not isinstance(obj, Obstacle):
            raise TypeError("Obstacle's only")
        assert obj.type in [MOVABLE, UNMOVABLE], f"added obstacle must have type MOVABLE or UNMOVABLE and is {obj.type}"
        #TODO check if the object is not already in the kgraph

        obj_node = ObstacleNode(self.unique_node_iden(), obj.name, obj)
        self.add_node(obj_node)


    def print_kgraph_info(self):
        """ print info of the kgraph. """
        print(f"info on kgraph nodes, n_nodes= {len(self.nodes)}")
        for node in self.nodes:
            print(f'node name: {node.obstacle.name}, iden: {node.iden}, type: {node.obstacle.type}')
        print(" ")

    def get_object_type(self, obj_name: str) -> int:
        """ return the type of the object if known. """


        for node in self.nodes:
            if node.obstacle.name == obj_name:
                return node.obstacle.type

        return None


# T5HIS IS SOME STUFF TO INITIALISE THE kgRAPH LATER
##  temp KGraph
            # kgraph = KGraph()
            # # the robot
            # node1 = ObjectSetNode(1, "robot", [])
            # kgraph.add_node(node1)
            # node2 = ChangeOfConfSetNode(2, "position", [])
            # kgraph.add_node(node2)
            # kgraph.add_edge(Edge("id", 1, 2, "MPC", "PEM"))
            #
            # self.kgraph = kgraph
            # self.kgraph.visualise(
            #     path="/home/gijs/Documents/semantic-thinking-robot/dashboard/data/knowledge_graph.html"

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


    def add_node(self, node):
        self.nodes.append(node)
