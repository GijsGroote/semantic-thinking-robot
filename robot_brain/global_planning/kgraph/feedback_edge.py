from abc import abstractmethod
import numpy as np
from robot_brain.controller.controller import Controller
from robot_brain.global_planning.edge import Edge
from robot_brain.local_planning.sample_based.motion_planner import MotionPlanner
from robot_brain.local_planning.graph_based.path_estimator import PathEstimator

from robot_brain.state import State
from robot_brain.object import Object

from robot_brain.global_planning.edge import EDGE_INITIALISED, EDGE_COMPLETED, EDGE_EXECUTING, EDGE_FAILED
from robot_brain.global_variables import DT

class FeedbackEdge(Edge):
    """ feedback edge containing feedback on a ActionEdge. """

    def __init__(self, iden: int,
            source: int,
            to: int,
            success_factor: float,
            robot_obj: Object,
            verb: str,
            controller: Controller,
            model_name: str):

        Edge.__init__(self, iden, source, to, verb, controller, "no_subtask")
        self.success_factor = success_factor
        self.robot_obj = robot_obj
        self.model_name = model_name

    @abstractmethod
    def respond(self) -> np.ndarray:
        """ respond to the current state. """

    @abstractmethod
    def view_completed(self) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """

    def completed(self) -> bool:
        """ returns true if the edge is completed, otherwise false. """
        return self.path_pointer >= len(self.motion_planner.shortest_path)-1

    def increment_current_view(self):
        """ updates toward the next current target from path. """

        if self.path_pointer < len(self.motion_planner.shortest_path)-1:
            self.path_pointer += 1

        if  len(self.motion_planner.shortest_path[self.path_pointer]) == 3:
            orien = self.motion_planner.shortest_path[self.path_pointer][2]
        else:
            orien = 0

        next_target = State(
                pos=np.array([self.motion_planner.shortest_path[self.path_pointer][0],
                    self.motion_planner.shortest_path[self.path_pointer][1], 0]),
                ang_p=np.array([0, 0, orien])
                )

        self.controller.set_target_state(next_target)

    def get_current_view(self) -> State:
        """ returns the current target the controller tries to steer toward. """
        return self.controller.target_state

    def create_log(self) -> dict:
        """ return a dictionary with metrics. """
        # TODO: maybe you want to be less strict here on that prediction error, gijs 11 nov 2022
        pred_error = self.controller.pred_error
        assert isinstance(pred_error, list),\
                f"pred_error should be a list and is {type(pred_error)}"
        assert all(isinstance(err, float) for err in pred_error),\
                "pred_error should contain only floats but does not"

        log = {
            "verb": self.verb,
            "controller_type": self.controller.name,
            "final_pose_error": None,
            }

        # TODO: this should be done differnetly, due to fist predition error begin 0 mostly
        if len(pred_error) > 1:
            log["avg_pred_error"] = np.round(np.average(pred_error), decimals=5)
            log["max_pred_error"] = np.round(max(pred_error), decimals=5)
            log["min_pred_error"] = np.round(min(pred_error[0:-1]), decimals=5)

        return log

    def ready_for_execution(self) -> bool:
        """ I am only here because my parent class enforces it. """
        return True

    def to_string(self):
        return f"Edge type: {type(self).__name__}<br>Edge identifier: {self.iden}"\
                f"Controller: {self.controller.name}<br>System model: {self.system_model.name}"

    @property
    def success_factor(self):
        return self._success_factor

    @success_factor.setter
    def success_factor(self, val):
        assert isinstance(val, str), f"success_factor should be a str and is {type(val)}"
        assert val >=0 and val <= 1, f"success_factor should be in interval [0, 1] and is{val}"
        self._success_factor = val


    @property
    def robot_obj(self):
        return self._robot_obj

    @robot_obj.setter
    def robot_obj(self, obj):
        assert isinstance(obj, Object), f"robot_obj should be an Object and is {type(obj)}"
        self._robot_obj = obj

    @property
    def model_name(self):
        return self._model_name

    @model_name.setter
    def model_name(self, val):
        assert isinstance(val, str), f"model_name should be a str and is {type(val)}"
        self._model_name = val

