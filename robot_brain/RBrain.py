import warnings

from robot_brain.planning.State import State
from robot_brain.Dynamics import Dynamics
# from robot_brain.controller.Mpc_old import Mpc
from robot_brain.planning.Object import Object
import numpy as np
from robot_brain.controller.mpc.Mpc import Mpc
from robot_brain.global_variables import *
from robot_brain.graph.HGraph import HGraph
from robot_brain.graph.KGraph import KGraph
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.graph.Edge import Edge
import pandas as pd
pd.options.plotting.backend = "plotly"
from casadi import *

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
        self.objects = dict()  # Object information dictionary
        self.verbs = []   # verbs which can be used
        self.robot = None  # Player information
        self.map = None  # Map used for planning
        self.is_doing = IS_DOING_NOTHING  # State indicating what the brain is doing
        self.controller = None

        self.height_of_map = 0  # Map height
        self.width_of_map = 0  # Map width
        self.defaultAction = None
        self.dt = None
        self.targetState = None
        self.hgraph = None
        self.kgraph = None

        # update all plots in webpage
        if CREATE_SERVER_DASHBOARD:
            from robot_brain.dashboard.app import startDashServer
            startDashServer()

    def setup(self, stat_world_info, ob):
        # create robot
        robot = Object("robot", State(pos=ob["joint_state"]["position"], vel=ob["joint_state"]["velocity"]), "urdf")
        # todo: this state above in incorrect for the x and xdot
        self.robot = robot
        self.objects["robot"] = robot

        # Create objects
        if "obstacleSensor" in ob.keys():
            for key, val in ob["obstacleSensor"].items():
                s_temp = State(pos=val["pose"]["position"],
                      vel=val["twist"]["linear"],
                      ang_p=val["pose"]["orientation"],
                      ang_v=val["twist"]["angular"])

                # s_temp = State(pos=val["x"][0:2], vel=val["xdot"][0:2], ang_p=val["x"][2], ang_v=val["xdot"][2])
                self.objects[key] = Object(key, s_temp, "urdf")

            self.action = np.array([0.0, 0.0])

        if "defaultAction" in stat_world_info.keys():
            self.defaultAction = stat_world_info["defaultAction"]


        self.dt = stat_world_info["dt"]
        self.targetState = stat_world_info["targetState"]

    def update(self, ob):
        """x
        Update all objects states
        :param ob:
        :return:
        """
        # update robot
        # todo: make this to better thingy if that is there
        pos = ob["joint_state"]["position"][0:2]
        vel = ob["joint_state"]["velocity"][0:2]
        
        self.robot.state.pos = np.array([pos[0], pos[1], 0])
        self.robot.state.vel = np.array([vel[0], vel[1], 0])

        self.robot.state.ang_p = ob["joint_state"]["position"][2]
        self.robot.state.ang_v = ob["joint_state"]["velocity"][2]

        # update objects
        if "obstacleSensor" in ob.keys():
            for key, val in ob["obstacleSensor"].items():
                if key in self.objects.keys():
                    self.objects[key].state.pos = val["pose"]["position"]
                    self.objects[key].state.vel = val["twist"]["linear"]
                    self.objects[key].state.ang_p = val["pose"]["orientation"]
                    self.objects[key].state.ang_v = val["twist"]["angular"]
        


    def respond(self):
        """ Respond to request with the latest action """

        if self.is_doing is IS_EXECUTING:
            # send action
            if self.controller is not None:

                self.timer = self.timer + 1
                # if self.timer == 75:
                #     self.hgraph.visualise("../robot_brain/dashboard/data/hgraph.html")
                #
                # if self.timer == 55:
                #     self.hgraph.visualise("../robot_brain/dashboard/data/hgraph.html")
                #
                # if self.timer == 25:
                #     self.hgraph.visualise("../robot_brain/dashboard/data/hgraph.html")
                #
                # if self.timer == 105:
                #     currentNode = ObjectSetNode(10, "PPPP", [])
                #     self.hgraph.addNode(currentNode)
                #     mpc_edge = Edge("15", 1, 10, "mpc", self.controller)
                #     self.hgraph.addEdge(mpc_edge)
                #     self.hgraph.visualise("../robot_brain/dashboard/data/hgraph.html")
                #
                return self.controller.respond(self.robot.state)

            else:
                warnings.warn("returning default action")
                return self.defaultAction


        elif self.is_doing is IS_DOING_NOTHING:
            return self.calculate_plan()

        else:
            raise Exception("Unable to respond")

        
    def calculate_plan(self):
        # set brain state to thinking
        self.timer = 0
        if self.hgraph is None:
            # THIS CHUNK OF STUFF IS WHAT SHOULD GO IN HGRAPH
            hgraph = HGraph()

            # adding nodes
            hgraph.addNode(ConfSetNode(1, "P", []))
            hgraph.addNode(ConfSetNode(2, "Pi", []))
            hgraph.addTargetNode(ConfSetNode(3, "Pis", []))
            hgraph.addStartNode(ObjectSetNode(4, "P", []))
            hgraph.addNode(ObjectSetNode(5, "P", []))

            hgraph.addEdge(Edge("id", 2, 3, "pid", "controller"))
            hgraph.addEdge(Edge("id", 5, 1, "pid", "controller"))
            hgraph.addEdge(Edge("id", 3, 1, "pid", "controller"))
            hgraph.addEdge(Edge("id", 3, 3, "EMPPI", "controller"))
            hgraph.addEdge(Edge("id", 4, 5, "mpc", "controller"))

            self.hgraph = hgraph
            # this hgraph is amazing, save it as html

            # self.hgraph.visualise("../robot_brain/dashboard/data/hgraph.html")

            kgraph = KGraph()

            # the robot
            node1 = ObjectSetNode(1, "robot", [])
            kgraph.addNode(node1)
            node2 = ChangeOfConfSetNode(2, "position", [])
            kgraph.addNode(node2)
            kgraph.addEdge(Edge("id", 1, 2, "MPC", "PEM"))
            # kgraph.addEdge(Edge("id", 1, 2, "MPC", "IPEM"))

            # adding expanded start and target node
            node3 = ObjectSetNode(3, "robot_and_red_sphere", [])
            node4 = ChangeOfConfSetNode(4, "box position", [])
            kgraph.addNode(node3)
            kgraph.addNode(node4)
            # kgraph.addNode(ObjectSetNode(5, "unknown_object", []))

            kgraph.addEdge(Edge("id", 3, 4, "EMPPI", "LSTM")) 

            self.kgraph = kgraph

            # self.kgraph.visualise("../robot_brain/dashboard/data/kgraph.html")
        print("yes I got it, MPC! executing plan")
        self.controller = Mpc()
        # dyn_model = Dynamics()
        # dyn_model.set_boxer_model()
        def dyn_model(x, u):
            dx_next = vertcat(
                    x[0] + 0.05*np.cos(x[2]) * u[0],
                    x[1] + 0.05*np.sin(x[2]) * u[0],
                    x[2] + 0.05 * u[1])
            return dx_next

        self.controller.setup(dyn_model, self.robot.state, self.targetState)

        self.is_doing = IS_EXECUTING

        return self.controller.respond(self.robot.state)





    # todo: all setters and getters should be sanitized properly, and test!




