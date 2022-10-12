# import pandas as pd
from abc import ABC, abstractmethod
import numpy as np
from pyvis.network import Network
from robot_brain.global_planning.graph import Graph
from robot_brain.global_planning.conf_set_node import ConfSetNode
from robot_brain.global_planning.object_set_node import ObjectSetNode
from robot_brain.global_planning.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.global_variables import FIG_BG_COLOR

from casadi import vertcat
from robot_brain.controller.mpc.mpc import Mpc
from robot_brain.global_planning.hgraph.local_planning.graph_based.rectangular_robot_occupancy_map import (
    RectangularRobotOccupancyMap,
)
from robot_brain.global_planning.hgraph.local_planning.graph_based.circular_robot_occupancy_map import (
    CircleRobotOccupancyMap,
)

from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.conf_set_node import ConfSetNode
from robot_brain.global_planning.object_set_node import ObjectSetNode
from robot_brain.global_planning.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.global_planning.edge import Edge
import math
from robot_brain.state import State

IN_EXECUTION_LOOP = "executing"
IN_SEARCH_HYPOTHESIS_LOOP = "searching"

class HGraph(Graph):
    """
    Hypothesis graph.
    """
    def __init__(self, robot):
        Graph.__init__(self)
        
        self.in_loop = IN_SEARCH_HYPOTHESIS_LOOP
        self.task = None
        self.target_nodes = []
        self.subtasks = []
        self.start_nodes = []
        self.current_node = None
        self.target_state = None
        self.controller = None
        self.hypothesis = []
        # todo: dependent on point_robot or boxer_robot, create an abstract class which handles that specific robot
        # USE INHERITANCE!!
        self.robot = robot
        if robot.name == "point_robot":
            self.robot_name = robot.name

        elif robot.name == "boxer_robot":
            self.robot_name = robot.name

        else:
            raise ValueError("unknown robot_type: {robot.name}")
       # TODO: task definition

        # TODO: keep track of the current state we are in

        # create controllers and pass them to

    def respond(self, current_state):
    
        if self.in_loop == IN_EXECUTION_LOOP:

            current_edge = self.hypothesis[0]
            # check if the next target state is almost reached
            if np.linalg.norm(current_state.get_xy_position() - current_edge.temp_target.get_xy_position()) < 0.5:

                if len(current_edge.path) != 0:

                    next_target = State(pos=np.array(current_edge.path[0][0:2]), ang_p=np.array([0, 0, current_edge.path[0][2]]))
                    print(f"target reached, now setting {next_target} as goal")
                    current_edge.temp_target = next_target
                    current_edge.controller.set_target_state(next_target)
                    current_edge.path = current_edge.path[1:]
                            
                else:
                    print("controller is done!!")
                    self.in_loop = IN_SEARCH_HYPOTHESIS_LOOP
                    raise StopIteration("The path has been completed allright")

            return current_edge.controller.respond(current_state)

        elif self.in_loop == IN_SEARCH_HYPOTHESIS_LOOP:
            return self.search_hypothesis()
        else:
            raise ValueError(f"HGraph in an unknown loop: {self.in_loop}")

    def search_hypothesis(self):

        # that while loop here

        # while len(subtasks) != 0:

            # find a start and target node to connect

            # check knowledge graph for information

            # create new transition/edge between these 2 nodes

            # if path extimation is true

            # if motion planning is done

            # if path to target is found

            # if first action is planned
            
            # go to execution state
 
        (self.robot, robot_target) = self.task[0] # that is the first subtask in the list
        path = self.estimate_robot_path_existance(robot_target, self.objects)
       
        # create controller
        self.target_state = robot_target
        self.create_controller()
        # path from occu graph
        edge = Edge(iden = 1,
                source = "start",
                to = "target",
                verb = "driving",
                controller = self.controller,
                path = path,
                )
        
        self.hypothesis = [edge]

        self.in_loop = IN_EXECUTION_LOOP

    def setup(self, task, objects):
        """ create start and target nodes
        TODO: this docstring """
        self.task = task
        self.objects = objects
        print(f"objects from setup in hgraph {self.objects}")
        self.search_hypothesis()
        # Temp hgraph here please
        # self.add_node(ConfSetNode(1, "P", []))
        # self.add_node(ConfSetNode(2, "Pi", []))
        # self.add_node(ConfSetNode(3, "Pis", []))
        #
        # self.add_start_node(ObjectSetNode(4, "P", []))
        # self.add_node(ObjectSetNode(5, "P", []))
        #
        # self.add_edge(Edge("id", 2, 3, "pid", "controller"))
        # self.add_edge(Edge("id", 5, 1, "pid", "controller"))
        # self.add_edge(Edge("id", 3, 1, "pid", "controller"))
        # self.add_edge(Edge("id", 3, 3, "EMPPI", "controller"))
        # self.add_edge(Edge("id", 4, 5, "mpc", "controller"))
        # self.visualise(
        #     path="/home/gijs/Documents/semantic-thinking-robot/dashboard/data/hgraph.html"
        # )
        
        # TODO: create occupancy map, and return controller
        

    @abstractmethod
    def estimate_robot_path_existance(self, target_state, objects):
        pass
        
    def create_controller(self):

        controller = Mpc()
        # dyn_model = Dynamics()
        # dyn_model.set_boxer_model()
        def dyn_model(x, u):
            dx_next = vertcat(
                x[0] + 0.05 * np.cos(x[2]) * u[0],
                x[1] + 0.05 * np.sin(x[2]) * u[0],
                x[2] + 0.05 * u[1],
            )
            return dx_next

        controller.setup(dyn_model, self.robot.state, self.target_state)
        
        self.controller = controller


    def plot_occupancy_graph(self, save=True):
        """plot the occupancy graph for the robot"""

        # if self.robot.name == "point_robot":
        #     self.occ_graph = CircleRobotOccupancyMap(1, 10, 12, self.objects, 1.1, self.robot.state.get_2d_pose())
        # elif self.robot.name == "boxer_robot":
        #     self.occ_graph = RectangularRobotOccupancyMap(1, 10, 12, self.objects, self.robot.state.get_2d_pose(), 1, 0.8, 0.5)
        # else:
        #     raise ValueError("unknown robot_type: {self.robot_type}")

        self.occ_graph = RectangularRobotOccupancyMap(1, 10, 12, {}, np.array([1,1,1]), 1, 0.8, 0.5)
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
                "/dashboard/assets/graph_template.html"

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
