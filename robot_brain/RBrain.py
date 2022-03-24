import warnings

from robot_brain.State import State
from robot_brain.Dynamics import Dynamics
# from robot_brain.controllers.Mpc_old import Mpc
from robot_brain.Object import Object
import numpy as np
from robot_brain.controllers.mpc.Mpc import Mpc
from robot_brain.global_variables import *
from robot_brain.graphs.HGraph import HGraph
from robot_brain.graphs.ConfSetNode import ConfSetNode
from robot_brain.graphs.ObjectSetNode import ObjectSetNode
from robot_brain.graphs.Edge import Edge
import pandas as pd
pd.options.plotting.backend = "plotly"


# is_doing states
IS_DOING_NOTHING = "nothing"
IS_THINKING = "thinking"
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
            # from robot_brain.dashboard.dashboard import Dashboard
            # db = Dashboard()
            # db.startDashServer()
            from robot_brain.dashboard.app import Dashboard
            db = Dashboard()
            db.startDashServer()

    def setup(self, stat_world_info, ob):
        # create robot
        robot = Object("robot", State(pos=ob["x"], vel=ob["xdot"]), "urdf")
        self.robot = robot
        self.objects["robot"] = robot

        # Create objects
        if "obstacleSensor" in ob.keys():
            for key, val in ob["obstacleSensor"].items():
                s_temp = State(pos=val["x"], vel=val["xdot"], ang_p=val["theta"], ang_v=val["thetadot"])
                self.objects[key] = Object(key, s_temp)

            self.action = np.array([0.0, 0.0])

        if "defaultAction" in stat_world_info.keys():
            self.defaultAction = stat_world_info["defaultAction"]


        self.dt = stat_world_info["dt"]
        self.targetState = stat_world_info["targetState"]


    def update(self, ob):
        """
        Update all objects states
        :param ob:
        :return:
        """
        # update robot
        self.robot.state.pos = ob["x"]
        self.robot.state.vel = ob["xdot"]
        # todo: update angular position and velocity
        # self.robot.state.ang_p = ob["obstacleSensor"]["0"]["theta"]
        # self.robot.state.ang_v = ob["obstacleSensor"]["0"]["thetadot"]


        # update objects
        if "obstacleSensor" in ob.keys():
            for key, val in ob["obstacleSensor"].items():
                self.objects[key].state.pos = val["x"]
                self.objects[key].state.vel = val["xdot"]
                self.objects[key].state.ang_p = val["theta"]
                self.objects[key].state.ang_v = val["thetadot"]
                # acceleration is not observed



    def respond(self):
        """ Respond to request with the latest action """


        if self.is_doing is IS_EXECUTING:
            # send action
            if self.controller is not None:

                return self.controller.respond(self.robot.state)
            else:
                warnings.warn("returning default action")
                return self.defaultAction

        elif self.is_doing is IS_THINKING:
            # todo: thinking should not really be a thing any more.
            # send action
            return self.defaultAction

        elif self.is_doing is IS_DOING_NOTHING:
            return self.calculate_plan()

        else:
            raise Exception("Unable to respond")



        
    def calculate_plan(self):
        # set brain state to thinking

        if self.hgraph is None:
            # THIS CHUNK OF STUFF IS WHAT SHOULD GO IN HGRAPH
            hgraph = HGraph()
            targetNode = ConfSetNode(999, "P", [])
            hgraph.addNode(targetNode)
            currentNode = ObjectSetNode(1, "P", [])
            hgraph.addNode(currentNode)
            self.hgraph = hgraph
            print("yes I got it, MPC! executing plan")
            self.controller = Mpc()
            dyn_model = Dynamics()
            dyn_model.set_boxer_model()
            # todo: this dyn model is unused
            self.controller.create_mpc_controller(self.dt, dyn_model, self.robot.state, self.targetState)
            mpc_edge = Edge("15", 1, 999, "mpc", self.controller)
            self.hgraph.addEdge(mpc_edge)

        self.is_doing = IS_EXECUTING

        return self.controller.respond(self.robot.state)





    # todo: all setters and getters should be sanitized properly, and test!




