import warnings
import numpy as np
import pandas as pd
from dashboard.app import start_dash_server
from robot_brain.state import State
from robot_brain.object import Object
from robot_brain.global_variables import CREATE_SERVER_DASHBOARD
from robot_brain.global_planning.hgraph.point_robot_hgraph import PointRobotHGraph
from robot_brain.global_planning.hgraph.boxer_robot_hgraph import BoxerRobotHGraph

import timeit
import math

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
        self.is_doing = IS_DOING_NOTHING  # State indicating what the brain is doing
        self.default_action = None
        self.dt = None
        self.hgraph = None
        self.kgraph = None
        self.obstacles_in_env = None

        # update all plots in webpage
        if CREATE_SERVER_DASHBOARD:
            start_dash_server()

    def visualise_grid(self):
        self.hgraph.plot_occupancy_graph()

    def setup(self, stat_world_info, ob):
        # create robot
        if "robot_type" in stat_world_info:
            if (
                stat_world_info["robot_type"] == "boxer_robot"
                or stat_world_info["robot_type"] == "point_robot"
            ):

                # TODO: detect which robot we are dealing with and give the robot some dimensions with robot.obstacle = obstacle..
                # that way I could access the robot's dimensions more easily

                self.robot = Object(
                    stat_world_info["robot_type"],
                    State(
                        pos=ob["joint_state"]["position"],
                        vel=ob["joint_state"]["velocity"],
                    ),
                    "urdf",
                )
            else:
                raise ValueError(
                    "only robot type 'boxer_robot' and 'point_robot' are allowed"
                )
        else:
            warnings.warn("robot type is not set")

        if "dt" in stat_world_info:
            self.dt = stat_world_info["dt"]
        else:
            warnings.warn("No DT found in static world info")

        if "obstacles_in_env" in stat_world_info:
            self.obstacles_in_env = stat_world_info["obstacles_in_env"]
        else:
            raise AttributeError(
                "there was no indication if this environment has obstacles"
            )

        if self.obstacles_in_env:
            assert (
                "obstacleSensor" in ob.keys()
            ), "no obstacle sensor found in initial observation"
            assert (
                "obstacles" in stat_world_info
            ), "no obstacle dict found in static world info"

            for key, val in ob["obstacleSensor"].items():

                s_temp = State(
                    pos=val["pose"]["position"],
                    vel=val["twist"]["linear"],
                    ang_p=val["pose"]["orientation"],
                    ang_v=val["twist"]["angular"],
                )
                self.objects[key] = Object(key, s_temp, "urdf")

                try:
                    self.objects[key].obstacle = stat_world_info["obstacles"][key]
                except KeyError as exc:
                    raise KeyError(
                        f"the obstacle {key} was returned from the obstacle sensor but not from the given obstacles"
                    ) from exc

        if "default_action" in stat_world_info.keys():
            self.default_action = stat_world_info["default_action"]

        # TODO: this should be a task, set of objects and target states
        if "target_state" in stat_world_info:
             # create HGraph with a (for now temporary task: placing the robot at some target location)
            self.target_state = stat_world_info["target_state"]
            self.is_doing = IS_EXECUTING
   
            if self.robot.name == "point_robot":
                self.hgraph = PointRobotHGraph(self.robot)
            elif self.robot.name == "boxer_robot":
                self.hgraph = BoxerRobotHGraph(self.robot)
            else:
                raise ValueError("unknown robot_type: {robot.name}")
 
            self.hgraph.setup(
                    [(self.robot, stat_world_info["target_state"])],
                self.objects)
        else:
            warnings.warn("no target state set")

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

    def respond(self):
        """
        Respond to request with the latest action.
        """
        if self.is_doing is IS_EXECUTING:
            if self.hgraph is not None:
                try:
                    return self.hgraph.respond(self.robot.state)
                except StopIteration as exc:
                    self.is_doing = IS_DOING_NOTHING
                    print(f"Stop with executing, because {exc}")
                    return self.default_action
            else:
                warnings.warn("returning default action")
                return self.default_action
        elif self.is_doing is IS_DOING_NOTHING:

            return self.default_action

        else:
            raise Exception("Unable to respond")

    # TODO: all setters and getters should be sanitized properly, and test!
