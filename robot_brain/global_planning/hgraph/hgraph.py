# import pandas as pd
from abc import ABC, abstractmethod
import numpy as np
import random
from pyvis.network import Network
from robot_brain.global_planning.graph import Graph
from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH
from casadi import vertcat

from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.node import Node
from robot_brain.global_planning.obstacle_node import ObstacleNode
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.obstacle import Obstacle
from robot_brain.global_planning.drive_identification_edge import DriveIdentificationEdge
from robot_brain.global_planning.drive_edge import DriveEdge
from robot_brain.controller.controller import Controller 
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
        self.subtasks = []
        self.initial_robot_node = None
        self.obstacles = {}
        self.start_nodes = []
        self.target_nodes = []
        self.start_to_target_iden = []
        self.current_node = None
        self.current_edge = DriveEdge(iden="tempEdge", 
                        source=1,
                        to=2,
                        verb="This edge should not be displayed",
                        controller=None,
                        path=[(1, 0)])

        self.hypothesis = []
        self.edge_pointer = 0

    def respond(self, current_state) -> np.ndarray:
        """ eventually responds with input for the robot. """

        if self.in_loop == IN_EXECUTION_LOOP:

            if len(self.hypothesis) == 0:
                raise ValueError("HGraph cannot respond without a hypothesis")

            self.current_edge = self.hypothesis[self.edge_pointer]

            # TODO: fault detection could be done here
            if np.linalg.norm(current_state.get_xy_position() - self.current_edge.get_current_target().get_xy_position()) < 0.5:

                if self.current_edge.completed():

                    if isinstance(self.current_edge, DriveIdentificationEdge):
                        self.update_system_model(self.current_edge, self.hypothesis[self.edge_pointer+1])

                    # what kind of edge was it?
                    # if self.get_node(edge.source) ==

                    if self.hypothesis_completed():

                        print("a hypothesis was completed!")
                        self.get_target_node(self.current_edge.to).completed = True
                        self.current_node = None
                        self.hypothesis = []
                        self.in_loop = IN_SEARCH_HYPOTHESIS_LOOP
                        self.edge_pointer = 0
                        self.search_hypothesis()

                        return self.respond(current_state)

                        # TODO: check if all subtasks are completed
                        # raise StopIteration("The path has been completed!")

                    else:
                        self.increment_edge_pointer()
                        self.visualise()
                        if isinstance(self.hypothesis[self.edge_pointer], DriveEdge):
                            if not self.hypothesis[self.edge_pointer].path:

                                self.search_path(self.hypothesis[self.edge_pointer])

                        return self.respond(current_state)

                else:
                    self.current_edge.increment_current_target()
            
            return self.current_edge.respond(current_state)

        elif self.in_loop == IN_SEARCH_HYPOTHESIS_LOOP:

            self.search_hypothesis()
            return self.respond(current_state)

        else:
            raise ValueError(f"HGraph in an unknown loop: {self.in_loop}")

    def search_hypothesis(self):
        """ Search by backward induction from a target node toward the robot start node. """

        hypothesis = []

        # this start node exist to compile the upcoming while loop 
        # TODO: write function which randomly selects a target node
        start_node = ObstacleNode(1000, "this_node_should_not_exist", Obstacle("This node should not exist", State(), "empty"))

        next_edge = None

        # search for unfinished target node and connect to starting node
        while start_node.name != self.robot.name:


            # find_subtask is not forced to find the same final_target_node
            (start_node, target_node) = self.find_subtask()
            
            # check for driving or pushing transition

            if start_node.name == self.robot.name and target_node.name == self.robot.name + "_target":
            
                # TODO: the knowledge graph should come into play here
                controller = self.create_driving_controller()
                
                model_node = ObstacleNode(self.unique_iden(), self.robot.name+"_model", self.robot)
                self.add_node(model_node)

                edge = DriveEdge(iden="iden", 
                        source=model_node.iden,
                        to=target_node.iden,
                        verb="driving",
                        controller=controller)


                # add edge to hgraph and hypothesis
                next_edge = edge
                self.add_edge(edge)
                hypothesis.insert(0, edge)

                # temp fix so the while loop keeps on looping
                start_node = ObstacleNode(1000, "this_node_should_not_exist", Obstacle("This node should not exist", State(), "empty"))

            elif start_node.name == self.robot.name and target_node.name == self.robot.name + "_model":

                # temporarily create a controller which drives closeby

                if next_edge.controller.name == "MPC":
                    controller = self._create_mpc_driving_controller()
                elif next_edge.controller.name == "MPPI":
                    controller = self._create_mppi_driving_controller()
                else:
                    raise ValueError("somethign goes terebly wrong")

                dyn_model = self.create_driving_model(controller.name)
                self._setup_driving_controller(controller, dyn_model)

                path = self.estimate_robot_path_existance(State(pos=np.array([-2.5, 0, 0])), self.obstacles)
                edge = DriveIdentificationEdge(iden="iden", 
                        source=start_node.iden,
                        to=target_node.iden,
                        verb="identify",
                        controller=controller,
                        path=path)

                # add edge to hgraph and hypothesis
                next_edge = edge
                self.add_edge(edge)
                hypothesis.insert(0, edge)

            elif start_node.name != self.robot.name and target_node.name != self.robot.name + "_target":
                # TODO: add node to drive the robot to the obstacle
                # TODO: Check for model available, add subtask system identification

                print('the start and target are both not the robot, thus pushing controller')

                # controller = self.create_driving_controller()
                # edge = pushg??Edge("iden", start_node.iden, target_node.iden, "driving", controller, path=path)

            else:
                raise ValueError("start and target nodes should be both the robot or both an obstacle")

        
        self.hypothesis = hypothesis
        self.current_node = self.get_start_node(self.hypothesis[0].source)
        self.in_loop = IN_EXECUTION_LOOP
        self.visualise()

    def update_system_model(self, current_edge, next_edge):
        """ update system model of the next edge. """
        # note this is now only for driving, not for pushign

        self._setup_driving_controller(next_edge.controller, current_edge.controller.dyn_model)        


    def search_path(self, edge):
        """ Search path for the edge """
        # TODO: is there a controller and a dynamic model 
        # TODO: this path existence check is a motion planner, create an actual motion planner
        target_node = self.get_node(edge.to)
        edge.path = self.estimate_robot_path_existance(target_node.obstacle.state, self.obstacles)


    def hypothesis_completed(self) -> bool:
        """ returns true if the hypothesis is completed, otherwise false. """
        return self.edge_pointer >= len(self.hypothesis)-1

    def increment_edge_pointer(self):
        """ updates toward the next edge in the hypothesis. """
        self.edge_pointer = self.edge_pointer + 1
        self.current_node = self.get_node(self.hypothesis[self.edge_pointer].source)
        


    def setup(self, task, obstacles):
        """ create start and target nodes. """

        self.task = task
        self.obstacles = obstacles

        #  add robot as start_state
        self.initial_robot_node = ObstacleNode(0, self.robot.name, self.robot)
        self.add_start_node(self.initial_robot_node)
        
        # For every task create a start and target node
        for (obst_temp, target) in task:
            iden_start_node = 0 # robots unique id
            if obst_temp != self.robot:
                iden_start_node = self.unique_iden()
                self.add_start_node(ObstacleNode(
                    iden_start_node,
                    obst_temp.name,
                    obst_temp,
                    ))
            iden_target_node = self.unique_iden()
            self.add_target_node(ObstacleNode(
                iden_target_node,
                obst_temp.name+"_target",
                Obstacle(obst_temp.name, target, obst_temp.properties),
                ))
            self.start_to_target_iden.append((iden_start_node, iden_target_node))


        self.visualise()
        self.search_hypothesis()
        
    def find_subtask(self) -> (Node, Node):
        """ returns 2 nodes in the hgraph to connect if these exist. """
        
        unfinished_target_nodes = [target_node for target_node in self.target_nodes if not target_node.completed]
        
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
                raise StopIteration("No more subtasks, No Solution Found!")
            else:
                start_node = self.get_start_node(self.get_start_iden_from_target_iden(robot_target_node.iden))
                final_target_node = robot_target_node
        else:
            start_node = self.get_start_node(self.get_start_iden_from_target_iden(obstacle_target_node.iden))
            final_target_node = obstacle_target_node


        edge_to_target_list = [edge for edge in self.edges if edge.to == final_target_node.iden]
        
        target_node = final_target_node

        # find iden of node point toward target, but nothing pointing toward itself
        while len(edge_to_target_list) != 0:
            # target_node = self.get_target_node(edge_to_target_list[0].to)
            target_node = self.get_node(edge_to_target_list[0].source)
            if start_node.iden == target_node.iden:
                start_node = self.initial_robot_node
                break
            edge_to_target_list = [edge for edge in self.edges if edge.to == edge_to_target_list[0].source]

        return (start_node, target_node)

    
    def create_driving_controller(self):
        """for now: randomly select a driving controller. """
        # TODO: find banned controllers, find blacklist, ask Kgraph for advice, 
        # fallback option is random select over all the availeble controllers

        possible_controllers = self.get_driving_controllers()
        
        controller = random.choice(possible_controllers)()

        return controller

    @abstractmethod
    def get_driving_controllers(self) -> list:
        pass

    @abstractmethod
    def create_driving_model(self, controller_name: str):
        """ create a dynamic model for driving. """
        pass

   
    def create_pushing_controller(self) -> Controller:
        # TODO: not implemented
        possible_controllers = self.get_pushing_controllers()
        return random.choice(possible_controllers)()

    @abstractmethod
    def get_pushing_controllers(self) -> list:
        pass

    @abstractmethod
    def create_pushing_model(self, controller_name: str):
        """ create a dynamic model for pushing. """
        pass

    @abstractmethod
    def estimate_robot_path_existance(self, target_state, obstacles):
        pass

    def unique_iden(self) -> int:
        """ return a unique identifyer. """
        iden = 0
        existing_idens = []

        for node in self.nodes:
            existing_idens.append(node.iden)

        while iden in existing_idens:
            iden += 1

        return iden

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

    def get_node(self, iden) -> Node:
        """ return  node by id, raises error if the identifyer does not exist. """
        node_list = [node for node in self.nodes if node.iden == iden]
        if len(node_list) == 0:
            raise IndexError(f"a node with identifyer {iden} does not exist.")
        else:
            return node_list[0]

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
        start_iden_list = [start_node[0] for start_node in self.start_to_target_iden if start_node[1] == target_iden]
        if len(start_iden_list) > 1:
            raise ValueError(f"there are {len(start_iden_list)} start node identifiers from target node iden {target_iden}")
        return start_iden_list[0]

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
        assert isinstance(edge.path, list), f"edge path is not a list but {type(edge.path)}"
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
