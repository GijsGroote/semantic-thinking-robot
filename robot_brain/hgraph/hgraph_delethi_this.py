import numpy as np

import math
from robot_brain.planning.graph_based.rectangular_robot_occupancy_map import (
    RectangularRobotOccupancyMap,
)
from robot_brain.planning.graph_based.circular_robot_occupancy_map import (
    CircleRobotOccupancyMap,
)


class Hgraph:


    def __init__(self, task):
        print("the initial Hgraph ! wow wieeeee")

        # TODO: task definition

        # TODO: keep track of the current state we are in

        # create controllers and pass them to

    def create_controller(self): 

        print("startig yo")
        if np.linalg.norm(self.robot.state.get_xy_position() - self.target_state.get_xy_position()) < 0.5:
        
        next_target = self.path[0]
        print(f"target reached, now setting {next_target} as goal")
        self.path = self.path[1:]
        self.target_state = State(pos=np.array(next_target[0:2]), ang_p=np.array([0, 0, next_target[2]]))
        self.controller.set_target_state(State(pos=np.array(next_target[0:2]), ang_p=np.array([0, 0, next_target[2]])))
            
            
        print(f"target state is: {self.target_state.to_string()}")
        return self.controller.respond(self.robot.state)


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

    def start(self):
        # adding nodes
        self.add_node(ConfSetNode(1, "P", []))
        self.add_node(ConfSetNode(2, "Pi", []))
        self.add_target_node(ConfSetNode(3, "Pis", []))
        self.add_start_node(ObjectSetNode(4, "P", []))
        self.add_node(ObjectSetNode(5, "P", []))

        self.add_edge(Edge("id", 2, 3, "pid", "controller"))
        self.add_edge(Edge("id", 5, 1, "pid", "controller"))
        self.add_edge(Edge("id", 3, 1, "pid", "controller"))
        self.hgraph.add_edge(Edge("id", 3, 3, "EMPPI", "controller"))
        self.hgraph.add_edge(Edge("id", 4, 5, "mpc", "controller"))

        # this hgraph is amazing, save it as html


