import warnings
import numpy as np
from casadi import vertcat
import pandas as pd
from robot_brain.dashboard.app import start_dash_server
from robot_brain.planning.state import State
from robot_brain.planning.object import Object
from robot_brain.controller.mpc.mpc import Mpc
from robot_brain.global_variables import CREATE_SERVER_DASHBOARD
from robot_brain.graph.h_graph import HGraph
from robot_brain.graph.k_graph import KGraph
from robot_brain.graph.conf_set_node import ConfSetNode
from robot_brain.graph.object_set_node import ObjectSetNode
from robot_brain.graph.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.graph.edge import Edge
pd.options.plotting.backend = "plotly"

# is_doing states
IS_DOING_NOTHING = "nothing"
IS_EXECUTING = "executing"


class RBrain:
    """
    RBrain Class

    Finds best input for the robot in order to reach a given task
    by learning world properties and minimizing cost function of given task
    """

    def __init__(self):
        # TODO: these should be private, en make some getters
        self.objects = {}  # Object information dictionary
        self.robot = None  # Player information
        self.controller = None
        self.is_doing = IS_DOING_NOTHING  # State indicating what the brain is doing
        self.default_action = None
        self.dt = None
        self.target_state = None # should become goal
        self.hgraph = None
        self.kgraph = None
        self.time_it = 0 # can be delted
        # update all plots in webpage
        if CREATE_SERVER_DASHBOARD:
            start_dash_server()

    def setup(self, stat_world_info, ob):
        # create robot
        robot = Object(
                "robot",
                State(pos=ob["joint_state"]["position"],
                    vel=ob["joint_state"]["velocity"]),
                "urdf")

        # TODO: this state above in incorrect for the x and xdot
        self.robot = robot
        self.objects["robot"] = robot

        # Create objects
        if "obstacleSensor" in ob.keys():
            for key, val in ob["obstacleSensor"].items():
                s_temp = State(pos=val["pose"]["position"],
                      vel=val["twist"]["linear"],
                      ang_p=val["pose"]["orientation"],
                      ang_v=val["twist"]["angular"])
                self.objects[key] = Object(key, s_temp, "urdf")

        # add static object info to the list of objects
        if "obstacles" in stat_world_info.keys():
            for (key, obst) in stat_world_info["obstacles"].items():
                try:
                    self.objects[obst.name()].obstacle = obst
                except KeyError:
                    raise KeyError("first add obstacles, then sensors, did you set_bullet_id_to_obst?")

        if "defaultAction" in stat_world_info.keys():
            self.default_action = stat_world_info["defaultAction"]

        self.dt = stat_world_info["dt"]
        self.target_state = stat_world_info["target_state"]

        

    def update(self, ob):
        """
        Update all objects states.
        :param ob:
        :return:
        """
        # update robot
        # TODO: make this to better thingy if that is there
        pos = ob["joint_state"]["position"][0:2]
        vel = ob["joint_state"]["velocity"][0:2]

        self.robot.state.pos = np.array([pos[0], pos[1], 0])
        self.robot.state.vel = np.array([vel[0], vel[1], 0])

        self.robot.state.ang_p = ob["joint_state"]["position"][2]
        self.robot.state.ang_v = ob["joint_state"]["velocity"][2]

        # update objects
        if "obstacleSensor" in ob.keys():
            for key, val in ob["obstacleSensor"].items():
                if key in self.objects:
                # if key in self.objects.keys():
                    self.objects[key].state.pos = val["pose"]["position"]
                    self.objects[key].state.vel = val["twist"]["linear"]
                    self.objects[key].state.ang_p = val["pose"]["orientation"]
                    self.objects[key].state.ang_v = val["twist"]["angular"]

        # this can be deleted
        self.time_it = self.time_it + 1
        if self.time_it == 150:
            self.hgraph.visualise("/home/gijs/Documents/semantic-thinking-robot/robot_brain/dashboard/data/hgraph.html")
            self.kgraph.visualise("/home/gijs/Documents/semantic-thinking-robot/robot_brain/dashboard/data/kgraph.html")


    def respond(self):
        """
        Respond to request with the latest action.
        """
        if self.is_doing is IS_EXECUTING:
            # send action
            if self.controller is not None:
                return self.controller.respond(self.robot.state)
            else:
                warnings.warn("returning default action")
                return self.default_action

        elif self.is_doing is IS_DOING_NOTHING:
            return self.calculate_plan()

        else:
            raise Exception("Unable to respond")

    def calculate_plan(self):
        # set brain state to thinking
        if self.hgraph is None:
            # THIS CHUNK OF STUFF IS WHAT SHOULD GO IN HGRAPH
            hgraph = HGraph()

            # adding nodes
            hgraph.add_node(ConfSetNode(1, "P", []))
            hgraph.add_node(ConfSetNode(2, "Pi", []))
            hgraph.add_target_node(ConfSetNode(3, "Pis", []))
            hgraph.add_start_node(ObjectSetNode(4, "P", []))
            hgraph.add_node(ObjectSetNode(5, "P", []))

            hgraph.add_edge(Edge("id", 2, 3, "pid", "controller"))
            hgraph.add_edge(Edge("id", 5, 1, "pid", "controller"))
            hgraph.add_edge(Edge("id", 3, 1, "pid", "controller"))
            hgraph.add_edge(Edge("id", 3, 3, "EMPPI", "controller"))
            hgraph.add_edge(Edge("id", 4, 5, "mpc", "controller"))

            self.hgraph = hgraph
            # this hgraph is amazing, save it as html

            self.hgraph.visualise("/home/gijs/Documents/semantic-thinking-robot/robot_brain/dashboard/data/hgraph.html")

            kgraph = KGraph()

            # the robot
            node1 = ObjectSetNode(1, "robot", [])
            kgraph.add_node(node1)
            node2 = ChangeOfConfSetNode(2, "position", [])
            kgraph.add_node(node2)
            kgraph.add_edge(Edge("id", 1, 2, "MPC", "PEM"))
            # kgraph.addEdge(Edge("id", 1, 2, "MPC", "IPEM"))

            # adding expanded start and target node
            node3 = ObjectSetNode(3, "robot_and_red_sphere", [])
            node4 = ChangeOfConfSetNode(4, "box position", [])
            kgraph.add_node(node3)
            kgraph.add_node(node4)
            # kgraph.addNode(ObjectSetNode(5, "unknown_object", []))

            kgraph.add_edge(Edge("id", 3, 4, "EMPPI", "LSTM"))

            self.kgraph = kgraph

            self.kgraph.visualise("/home/gijs/Documents/semantic-thinking-robot/robot_brain/dashboard/data/kgraph.html")
        self.controller = Mpc()
        # dyn_model = Dynamics()
        # dyn_model.set_boxer_model()
        def dyn_model(x, u):
            dx_next = vertcat(
                    x[0] + 0.05*np.cos(x[2]) * u[0],
                    x[1] + 0.05*np.sin(x[2]) * u[0],
                    x[2] + 0.05 * u[1])
            return dx_next

        self.controller.setup(dyn_model, self.robot.state, self.target_state)

        self.is_doing = IS_EXECUTING

        return self.controller.respond(self.robot.state)

    # TODO: all setters and getters should be sanitized properly, and test!
