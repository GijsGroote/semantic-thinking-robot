# import pandas as pd
import numpy as np
from pyvis.network import Network
from robot_brain.graph.graph import Graph
from robot_brain.graph.conf_set_node import ConfSetNode
from robot_brain.graph.object_set_node import ObjectSetNode
from robot_brain.graph.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.global_variables import FIG_BG_COLOR
from robot_brain.planning.graph_based.rectangular_robot_occupancy_map import (
    RectangularRobotOccupancyMap,
)
from robot_brain.planning.graph_based.circular_robot_occupancy_map import (
    CircleRobotOccupancyMap,
)


class HGraph(Graph):
    """
    Hypothesis graph.
    """
    def __init__(self):
        Graph.__init__(self)
        self.target_nodes = []
        self.start_nodes = []
        self.current_node = None

       # TODO: task definition

        # TODO: keep track of the current state we are in

        # create controllers and pass them to

    def create_controller(self): 
        
        # TODO: setup beginning functionality. Hgraph should eventually only give the controller, that will be all
        print("startig yo")
        # if np.linalg.norm(self.robot.state.get_xy_position() - self.target_state.get_xy_position()) < 0.5:
        # 
        # next_target = self.path[0]
        # print(f"target reached, now setting {next_target} as goal")
        # self.path = self.path[1:]
        # self.target_state = State(pos=np.array(next_target[0:2]), ang_p=np.array([0, 0, next_target[2]]))
        # self.controller.set_target_state(State(pos=np.array(next_target[0:2]), ang_p=np.array([0, 0, next_target[2]])))
        #     
        #     
        # print(f"target state is: {self.target_state.to_string()}")
        # return self.controller.respond(self.robot.state)


    def plot_occupancy_graph(self, save=True):
        """plot the occupancy graph for the robot"""

        if self.robot.name == "point_robot":
            self.occ_graph = CircleRobotOccupancyMap(1, 10, 12, self.objects, 1.1, self.robot.state.get_2d_pose())
        elif self.robot.name == "boxer_robot":
            self.occ_graph = RectangularRobotOccupancyMap(1, 10, 12, self.objects, self.robot.state.get_2d_pose(), 1, 0.8, 0.5)
        else:
            raise ValueError("unknown robot_type: {self.robot_type}")

        self.occ_graph.setup()
        self.occ_graph.visualise(save=save)



    def visualise(self, path=None):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """
        if path is None:
            bgcolor = None
        else:
            bgcolor = FIG_BG_COLOR

        net = Network(bgcolor=bgcolor, height="450px", directed=True)

        # set a custom style sheet
        net.path = "/home/gijs/Documents/semantic-thinking-robot"\
                "/robot_brain/dashboard/assets/graph_template.html"

        net.set_edge_smooth('dynamic')

        for node in self.start_nodes:
            if node == self.current_node:
                continue
            net.add_node(node.iden,
                    title = "Starting Node:<br>" + node.to_string() + "<br>",
                    x=1.0,
                    y=1.0,
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
                    title = "Target Node:<br>" + node.to_string() + "<br>",
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
                    title = "Node:<br>" + node.to_string() + "<br>",
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
                    title = "Current node:<br>" + self.current_node.to_string() + "<br>",
                    x=10.0,
                    y=10.0,
                    color= {
                        'border': '#fb4b50', # red
                        'background': '#fb7e81',
                        'highlight': {
                            'border': '#fb4b50',
                            'background': '#fcbcc4'
                            }
                        },label = self.current_node.name,
                    group = "current_node")


        # add edges
        for edge in self.edges:

            dashes = False
            if edge.path is False:
                dashes = True

            net.add_edge(edge.source,
                    edge.to,
                    weight=1.0,
                    dashes=dashes,
                    label=edge.verb,
                    title="edge:<br>" + edge.to_string() + "<br>",
                    )

        # if you want to edit cusomize the graph
        # net.show_buttons(filter_=['physics'])

        if path is None:
            net.show("delete.html")
        else:
            net.write_html(path)

    def add_node(self, node):
        if isinstance(node, ChangeOfConfSetNode):
            raise TypeError("ChangeOfConfSetNode's are not allowed in HGraph")
        self.nodes.append(node)

    def add_start_node(self, node):
        if not isinstance(node, ObjectSetNode):
            raise TypeError("ObjectSetNode's are only allowed as starting node in HGraph")
        self.start_nodes.append(node)

    def add_target_node(self, node):
        if not isinstance(node, ConfSetNode):
            raise TypeError("ConfSetNode's are only allowed as target node in HGraph")
        self.target_nodes.append(node)
