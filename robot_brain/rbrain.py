import warnings
import time
import numpy as np
import pandas as pd
from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle
pd.options.plotting.backend = "plotly"
from dashboard.app import start_dash_server, stop_dash_server
from robot_brain.state import State
from robot_brain.object import Object
from robot_brain.object import Object, MOVABLE, UNMOVABLE, UNKNOWN
from robot_brain.global_variables import CREATE_SERVER_DASHBOARD, POINT_ROBOT_RADIUS, BOXER_ROBOT_LENGTH, BOXER_ROBOT_WIDTH

from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.hgraph.halgorithm import HypothesisAlgorithm

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
        self.objects = {} # object information dictionary
        self.robot_obj = None  # Player information
        self.is_doing = IS_DOING_NOTHING  # State indicating what the brain is doing
        self.default_action = None
        self.dt = None
        self.halgorithm = None
        self.objects_in_env = None
        self.dash_app = None


    def setup(self, stat_world_info, ob):

        # update all plots in webpage
        if CREATE_SERVER_DASHBOARD:
            self.dash_app = start_dash_server(stat_world_info["n_env"])

        # create robot
        if "robot_type" in stat_world_info:
            # create shape of the robot
            if stat_world_info["robot_type"] == "pointRobot-vel-v7" or stat_world_info["robot_type"] == "pointRobot-acc-v7":
                cylinder_dict = {
                    "type": "box",
                    # "geometry": {"radius": 0.22, "height": 0.25},
                    "geometry": {"radius": POINT_ROBOT_RADIUS, "height": 0.25}, }
                robot_properties = CylinderObstacle(name="pointRobot-vel-v7-obst", content_dict=cylinder_dict)

            elif stat_world_info["robot_type"] == "boxerRobot-vel-v7-obst" or stat_world_info["robot_type"] == "boxerRobot-acc-v7":

                box_dict = { "type": "box",
                    "position": [0, 0, 0],
                    "geometry": {"length": BOXER_ROBOT_LENGTH, "width": BOXER_ROBOT_WIDTH, "height": 0.2},
                }
                robot_properties = BoxObstacle(name="box_robot", content_dict=box_dict)

            else:
                raise ValueError("unknown robot_type: {stat_world_info['robot_type']}")

            self.robot_obj = Object(
                name=stat_world_info["robot_type"],
                state=State(
                    pos=ob["joint_state"]["position"],
                    vel=ob["joint_state"]["velocity"],
                ),
                properties=robot_properties,
                obj_type=MOVABLE
            )
        else:
            warnings.warn("robot type is not set")

        if "dt" in stat_world_info:
            self.dt = stat_world_info["dt"]
        else:
            warnings.warn("No DT found in static world info")

        if "default_action" in stat_world_info.keys():
            self.default_action = stat_world_info["default_action"]


        if "objects_in_env" in stat_world_info:
            self.objects_in_env = stat_world_info["objects_in_env"]
            if self.objects_in_env:
                self.setup_objects(stat_world_info, ob)
        else:
            raise AttributeError(
                "there was no indication if this environment has objects"
            )

        if "task" in stat_world_info:
            self.setup_halgorithm(stat_world_info)
        else:
            warnings.warn("no task was set")

    def setup_objects(self, stat_world_info, ob):
        """ save objects and their dimensions. """

        assert (
            "obstacleSensor" in ob.keys()
        ), "no obstacle sensor found in initial observation"
        assert (
            "objects" in stat_world_info
        ), "no object dict found in static world info"

        for key, val in ob["obstacleSensor"].items():

            s_temp = State(
                pos=val["pose"]["position"],
                vel=val["twist"]["linear"],
                ang_p=val["pose"]["orientation"],
                ang_v=val["twist"]["angular"],
            )

            try:
                self.objects[key] = Object(name=key,
                            state=s_temp,
                            properties=stat_world_info["objects"][key])

            except KeyError as exc:
                raise KeyError(
                    f"the object {key} was returned from the\
                     object sensor but not from the given objects"
                ) from exc

            # objects are classified as unknown
            self.objects[key].type = UNKNOWN

    def setup_halgorithm(self, stat_world_info):
        """
        Setup hypothesis algorithm initialised with the task.

        2 types pointRobot-vel-v7
                boxerRobot-vel-v7

        of robots are allowed.
        """

        self.halgorithm = HypothesisAlgorithm(self.robot_obj, stat_world_info["env"])
        self.is_doing = IS_EXECUTING

        # halt if there are no subtask
        if len(stat_world_info["task"]) == 0:
            self.is_doing = IS_DOING_NOTHING

            if CREATE_SERVER_DASHBOARD:
                self.hgraph.visualise()
                stop_dash_server(self.dash_app)

        # create a task
        task = {}
        for (task_nmr, (object_key, target)) in enumerate(stat_world_info["task"]):

            if object_key == "robot":
                obj = self.robot_obj
            else:
                obj = self.objects[object_key]

            assert isinstance(target, State), \
            f"the target should be a State object and is: {type(target)}"

            assert isinstance(obj, Object), \
                    f"the obj should be of type Object and is {type(obj)}"

            task["subtask_"+str(task_nmr)] = (obj, target)

        if "kgraph" in stat_world_info:
            kgraph = stat_world_info["kgraph"]
        else:
            kgraph = KGraph()

        self.halgorithm.setup(
                kgraph=kgraph,
                task=task,
                objects=self.objects)

    def update(self, ob):
        """
        Update all objects states.
        :param ob:
        :return:
        """
        # update robot
        pos = ob["joint_state"]["position"][0:2]
        vel = ob["joint_state"]["velocity"][0:2]

        self.robot_obj.state.pos = np.array([pos[0], pos[1], 0])
        self.robot_obj.state.vel = np.array([vel[0], vel[1], 0])

        self.robot_obj.state.ang_p = ob["joint_state"]["position"][2]
        self.robot_obj.state.ang_v = ob["joint_state"]["velocity"][2]

        # update objects
        if "objectSensor" in ob.keys():
            for key, val in ob["obstacleSensor"].items():
                if key in self.objects:
                    # if key in self.object.keys():
                    self.objects[key].state.pos = val["pose"]["position"]
                    self.objects[key].state.vel = val["twist"]["linear"]
                    self.objects[key].state.ang_p = val["pose"]["orientation"]
                    self.objects[key].state.ang_v = val["twist"]["angular"]

    def respond(self):
        """
        Respond to request with the latest action.
        """
        if self.is_doing is IS_EXECUTING:
            if self.halgorithm is not None:
                try:
                    return self.halgorithm.respond()
                except StopIteration as exc:
                    self.is_doing = IS_DOING_NOTHING

                    print(f"Halt because: {exc}")

                    self.halgorithm.visualise()

                    if CREATE_SERVER_DASHBOARD:
                        self.halgorithm.visualise()
                        time.sleep(2) # give the dashboard some time to process visualising the hgraph
                        stop_dash_server(self.dash_app)

                    raise exc
            else:
                return self.default_action
        elif self.is_doing is IS_DOING_NOTHING:

            return self.default_action
        else:
            raise Exception("Unable to respond")

    @property
    def objects(self):
        return self._objects


    @objects.setter
    def objects(self, objects):
        assert isinstance(objects, dict), \
                f"objects should be a dictionary and is {type(objects)}"
        self._objects = objects
