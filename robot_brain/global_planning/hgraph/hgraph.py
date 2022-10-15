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

from robot_brain.global_planning.node import Node
from robot_brain.global_planning.conf_set_node import ConfSetNode
from robot_brain.global_planning.object_set_node import ObjectSetNode
from robot_brain.global_planning.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.configuration import Configuration
from robot_brain.object import Object
from robot_brain.global_planning.edge import Edge
import math
from robot_brain.state import State

IN_EXECUTION_LOOP = "executing"
IN_SEARCH_HYPOTHESIS_LOOP = "searching"

class HGraph(Graph):
    """
    Hypothesis graph.
    """
    def __init__(self):
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
        self.hypothesis_pointer = 0

        # TODO: task definition

        # TODO: keep track of the current state we are in

        # create controllers and pass them to

    def respond(self, current_state):
    
        if self.in_loop == IN_EXECUTION_LOOP:

            current_edge = self.hypothesis[self.hypothesis_pointer]

            # check if the next target state is almost reached
            if np.linalg.norm(current_state.get_xy_position() - current_edge.get_current_target().get_xy_position()) < 0.5:

                if current_edge.completed():
                    print(f"the length of hyptesis {len(self.hypothesis)} and the pointer {self.hypothesis_pointer}")
                    if self.hypothesis_completed():
                        print("a hypothesis was completed!@")
                        self.get_target_node(current_edge.to).completed = True
                        self.in_loop = IN_SEARCH_HYPOTHESIS_LOOP
                        
                        # TODO: check if all subtasks are completed
                        # raise StopIteration("The path has been completed!")

                    else:
                        self.increment_hypothesis_pointer()

                else:
                    current_edge.increment_current_target()

            return current_edge.respond(current_state)

        elif self.in_loop == IN_SEARCH_HYPOTHESIS_LOOP:
            print("searching a new hypothesis")
            self.search_hypothesis()

            return np.zeros(2)
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

        # (start_node, target_node) = self.find_subtask()
        #
        # print(f"found 2 nodes, connect iden {start_node}, to {target_node}")
        #
        # self.add_edge(Edge("identifyer", start_node.iden, target_node.iden, "drivnig?", []))
        #  
        # # Manually set that target to completed
        # target_node.completed = True
        #
        #
        start_node = ObjectSetNode(100, "temp_noide", [Object("naaam", State(), "urdf")])
        hypothesis = []
        
        # for target in self.target_nodes:
        #     print(f"all nodes names : {target.name}")
        while start_node.name != self.robot.name:

            (start_node, target_node) = self.find_subtask()
            
            print(f"This should be robot iden {start_node.iden}, to {target_node.iden}")

            path = self.estimate_robot_path_existance(target_node.object_set[0].state, self.objects)
            print(f"the path {path}")
            controller = self.create_controller()
            edge = Edge("iden", start_node.iden, target_node.iden, "driving", controller, path=path)
        
            self.add_edge(edge)
            hypothesis.append(edge)

        print(f"found a hypothesis which drives to {self.get_target_node(edge.to).object_set[0].state.pos}")
        self.hypothesis = hypothesis
        

        self.visualise(path="/home/gijs/Documents/semantic-thinking-robot/dashboard/data/hypothesis_graph.html")
        self.in_loop = IN_EXECUTION_LOOP

        

    def hypothesis_completed(self) -> bool:
        """ returns true if the hypothesis is completed, otherwise false. """
        return self.hypothesis_pointer >= len(self.hypothesis)-1

    def increment_hypothesis_pointer(self):
        """ updates toward the next edge in the hypothesis. """
        if self.hypothesis_pointer < len(self.hypothesis)-1:
            self.hypothesis_pointer += 1

    def setup(self, task, objects):
        """ create start and target nodes. """

        self.task = task
        self.objects = objects

        # always add robot as start_state
        self.add_start_node(ObjectSetNode(1, self.robot.name, [self.robot]))

        for (object_temp, target) in task:
            if object_temp != self.robot:
                self.add_start_node(ObjectSetNode(self.unique_iden(), object_temp.name, [object_temp]))

            self.add_target_node(ObjectSetNode(self.unique_iden(), object_temp.name+"_target", [Object(object_temp.name, target, "urdf")]))

        self.visualise(path="/home/gijs/Documents/semantic-thinking-robot/dashboard/data/hypothesis_graph.html")

        self.search_hypothesis()
       
        
    def unique_iden(self) -> int:
        """ return a unique identifyer. """
        iden = 0
        existing_idens = []

        for node in self.nodes:
            existing_idens.append(node.iden)

        while iden in existing_idens:
            iden += 1

        return iden


    def find_subtask(self) -> (Node, Node):
        """ returns 2 nodes in the hgraph to connect if these exist. """
        
        unfinished_target_nodes = [target_node for target_node in self.target_nodes if not target_node.completed]
        
        obstacle_target_node = None
        robot_target_node = None

        for unfinished_target_node in unfinished_target_nodes:
            if unfinished_target_node.name == self.robot.name + "_target":
                robot_target_node = unfinished_target_node
            else:
                obstacle_target_node = unfinished_target_node

        if obstacle_target_node is None:
            if robot_target_node is None:
                raise StopIteration("No more subtasks, No Solution Found!")
            else:
                # TODO A better check to see if start and target belong to the same object --> compare the  object itself
                # next line assumes a start_node.name + 7 letters == target_node.name, notice "_target" are 7 letters
                start_node = [start_node for start_node in self.start_nodes if start_node.name  == robot_target_node.name[0:-7]][0]
                final_target_node = robot_target_node
        else:
            start_node = [start_node for start_node in self.start_nodes if start_node.name == obstacle_target_node.name[0:-7]][0]
            final_target_node = obstacle_target_node


        edge_to_target_list = [edge for edge in self.edges if edge.to == final_target_node.iden]
        target_node = final_target_node
        
        # find iden of node point toward target, but nothing pointing toward itself
        while len(edge_to_target_list) != 0:
            target_node = self.get_target_node(edge_to_target_list[0].to)
            edge_to_target_list = [edge for edge in self.edges if edge.to == edge_to_target_list[0].source]

        return (start_node, target_node)


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

        controller.setup(dyn_model, self.robot.state, self.robot.state)
        
        return controller


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
                    title = f"Starting Node: {node.iden}<br>{node.to_string()}<br>",
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
            print("now visualising the plot")
            net.show("delete.html")
        else:
            net.write_html(path)
    
    def get_target_node(self, iden) -> Node:
        """ return target node by id, raises error if the identifyer does not exist. """
        
        target_node_list = [node for node in self.target_nodes if node.iden  == iden]

        if len(target_node_list) == 0:
            raise IndexError(f"a target node with identifyer {iden} does not exist.")
        else:
            return target_node_list[0]

    def add_node(self, node):
        if isinstance(node, ChangeOfConfSetNode):
            raise TypeError("ChangeOfConfSetNode's are not allowed in HGraph")
        self.nodes.append(node)

    def add_start_node(self, node):
        if not isinstance(node, ObjectSetNode):
            raise TypeError("ObjectSetNode's are only allowed as starting node in HGraph")
        self.add_node(node)
        self.start_nodes.append(node)

    def add_target_node(self, node):
        if isinstance(node, ChangeOfConfSetNode):
            raise TypeError("ChangeOfConfSetNode's are not allowed as target node in HGraph")

        # Configurations are perhaps not needed. Work with states, any unknown position, orientation is set to 0, 15 oct, Gijs Groote
        # if not isinstance(node, ConfSetNode):
        #     raise TypeError("ConfSetNode's are only allowed as target node in HGraph")

        self.add_node(node)
        self.target_nodes.append(node)

        # todo: dependent on point_robot or boxer_robot, create an abstract class which handles that specific robot

    @abstractmethod
    def robot(self):
        pass

