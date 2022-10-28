import warnings
import numpy as np
import pandas as pd
from dashboard.app import start_dash_server
from robot_brain.state import State
from robot_brain.obstacle import Obstacle
from robot_brain.global_variables import CREATE_SERVER_DASHBOARD
from robot_brain.global_planning.hgraph.point_robot_hgraph import PointRobotHGraph
from robot_brain.global_planning.hgraph.boxer_robot_hgraph import BoxerRobotHGraph


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
        self.obstacles = {} # obstacle information dictionary
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
            # TODO: detect which robot we are dealing with and give the 
            # robot some dimensions with robot.obstacle = obstacle..
            # that way I could access the robot's dimensions more easily

            self.robot = Obstacle(
                name=stat_world_info["robot_type"],
                state=State(
                    pos=ob["joint_state"]["position"],
                    vel=ob["joint_state"]["velocity"],
                ),
                properties="empty",
            )
        else:
            warnings.warn("robot type is not set")

        if "dt" in stat_world_info:
            self.dt = stat_world_info["dt"]
        else:
            warnings.warn("No DT found in static world info")

        if "default_action" in stat_world_info.keys():
            self.default_action = stat_world_info["default_action"]


        if "obstacles_in_env" in stat_world_info:
            self.obstacles_in_env = stat_world_info["obstacles_in_env"]
            if self.obstacles_in_env:
                self.setup_obstacles(stat_world_info, ob)
        else:
            raise AttributeError(
                "there was no indication if this environment has obstacles"
            )

        if "task" in stat_world_info:
            self.setup_task(stat_world_info)
        else:
            warnings.warn("no task was set")

    def setup_obstacles(self, stat_world_info, ob):
        """ save obstacles and their dimensions. """

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

            try:
                self.obstacles[key] = Obstacle(name=key,
                            state=s_temp,
                            properties=stat_world_info["obstacles"][key])

            except KeyError as exc:
                raise KeyError(
                    f"the obstacle {key} was returned from the\
                     obstacle sensor but not from the given obstacles"
                ) from exc

            # pretent that obstacles are unmovable, falsely mislabeled
            # PRENTEND THAT THIS OBSTACLE IS UNMOVABLE
            self.obstacles[key].type = "unmovable"


    def setup_task(self, stat_world_info):
        """ Setup Hypothesis graph initialised with the task. """

        if (stat_world_info["robot_type"] == "pointRobot-vel-v7" or
                stat_world_info["robot_type"] == "pointRobot-acc-v7"):
            self.hgraph = PointRobotHGraph(self.robot)

        elif (stat_world_info["robot_type"] == "boxerRobot-vel-v7" or
                stat_world_info["robot_type"] == "boxerRobot-acc-v7"):
            self.hgraph = BoxerRobotHGraph(self.robot)

        else:
            raise ValueError("unknown robot_type: {stat_world_info['robot_type']}")

        task = []
        for (obstacle_key, target) in stat_world_info["task"]:

            if obstacle_key == "robot":
                obstacle = self.robot
            else:
                obstacle = self.obstacles[obstacle_key.name()]

            assert isinstance(target, State), \
            f"the target should be a State object and is: {type(target)}"

            assert isinstance(obstacle, Obstacle), \
                    f"the obstacle should be of type Ostacle and in {type(obstacle)}"

            task.append((obstacle, target))

        self.hgraph.setup(
                task=task,
                obstacles=self.obstacles)

        self.is_doing = IS_EXECUTING

    def update(self, ob):
        """
        Update all obstacle states.
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

        # update obstacles
        if "obstacleSensor" in ob.keys():
            for key, val in ob["obstacleSensor"].items():
                if key in self.obstacles:
                    # if key in self.obstacle.keys():
                    self.obstacles[key].state.pos = val["pose"]["position"]
                    self.obstacles[key].state.vel = val["twist"]["linear"]
                    self.obstacles[key].state.ang_p = val["pose"]["orientation"]
                    self.obstacles[key].state.ang_v = val["twist"]["angular"]

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

                    self.hgraph.visualise()
                    return self.default_action
            else:
                warnings.warn("returning default action")
                return self.default_action
        elif self.is_doing is IS_DOING_NOTHING:

            return self.default_action

        else:
            raise Exception("Unable to respond")

    # TODO: all setters and getters should be sanitized properly, and test!
    @property
    def obstacles(self):
        return self._obstacles


    @obstacles.setter
    def obstacles(self, obstacles):
        assert isinstance(obstacles, dict), \
                f"obstacles should be a dictionary and is {type(obstacles)}"
        self._obstacles = obstacles
