import warnings

from robot_brain.planning.State import State
from robot_brain.Dynamics import Dynamics
# from robot_brain.controllers.Mpc_old import Mpc
from robot_brain.planning.Object import Object
import numpy as np
from robot_brain.controllers.mpc.Mpc import Mpc
from robot_brain.global_variables import *
from robot_brain.graph.HGraph import HGraph
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.graph.Edge import Edge
import pandas as pd
pd.options.plotting.backend = "plotly"
from robot_brain.dashboard.figures import create_graph_plot

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
        self.robot.state.pos = ob["x"][0:2]
        self.robot.state.vel = ob["xdot"][0:2]
        self.robot.state.ang_p = ob["x"][2]
        self.robot.state.ang_v = ob["xdot"][2]

        # update objects
        if "obstacleSensor" in ob.keys():
            for key, val in ob["obstacleSensor"].items():
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
                if self.timer == 75:
                    create_graph_plot(self.hgraph, "../robot_brain/dashboard/data/hgraph.html")

                if self.timer == 55:
                    create_graph_plot(self.hgraph, "../robot_brain/dashboard/data/hgraph.html")

                if self.timer == 25:
                    create_graph_plot(self.hgraph, "../robot_brain/dashboard/data/hgraph.html")

                if self.timer == 105:
                    currentNode = ObjectSetNode(10, "PPPP", [])
                    self.hgraph.addNode(currentNode)
                    mpc_edge = Edge("15", 1, 10, "mpc", self.controller)
                    self.hgraph.addEdge(mpc_edge)
                    create_graph_plot(self.hgraph, "../robot_brain/dashboard/data/hgraph.html")

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
            create_graph_plot(hgraph, "../robot_brain/dashboard/data/hgraph.html")


            print("yes I got it, MPC! executing plan")
            self.controller = Mpc()
            dyn_model = Dynamics()
            dyn_model.set_boxer_model()
            # todo: this dyn model is unused
            self.controller.create_mpc_controller(self.dt, dyn_model, self.robot.state, self.targetState)

        self.is_doing = IS_EXECUTING

        return self.controller.respond(self.robot.state)





    # todo: all setters and getters should be sanitized properly, and test!




