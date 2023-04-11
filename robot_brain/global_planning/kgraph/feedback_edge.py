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

    def __init__(self,
            iden: int,
            source: int,
            to: int,
            success_factor: float,
            obj: Object,
            verb: str,
            controller: Controller,
            model_name: str,
            edge_status):

        Edge.__init__(self, iden, source, to, verb, controller, "no_subtask")
        self.success_factor = success_factor
        self.n_failed = 0
        self.n_success = 0
        self.obj = obj
        self.status = EDGE_INITIALISED
        self.model_name = model_name
        if edge_status == EDGE_COMPLETED:
            self.n_success = 1
        elif edge_status == EDGE_FAILED:
            self.n_failed = 1
        else:
            raise ValueError(f"incorrect edge_status encountered: {edge_status}")

    def completed(self) -> bool:
        pass

    def increment_current_view(self):
        pass

    def create_log(self) -> dict:
        pass
    def ready_for_execution(self) -> bool:
        pass

    def to_string(self):
        return f"Edge type: {type(self).__name__}<br>Edge identifier: {self.iden}<br>"\
                f"Object: {self.obj.name},<br>Success Factor: {self.success_factor}<br>"\
                f"Controller: {self.controller.name}<br>System model: {self.model_name}<br>"\
                f"n_success: {self.n_success}<br>n_failed: {self.n_failed}"

    @property
    def success_factor(self):
        return self._success_factor

    @success_factor.setter
    def success_factor(self, val):
        assert isinstance(val, float), f"success_factor should be a float and is {type(val)}"
        assert 0.0 <= val <= 1.0, f"success_factor should be in interval [0, 1] and is{val}"
        self._success_factor = val

    @property
    def obj(self):
        return self._obj

    @obj.setter
    def obj(self, obj):
        assert isinstance(obj, Object), f"obj should be an Object and is {type(obj)}"
        self._obj = obj

    @property
    def model_name(self):
        return self._model_name

    @model_name.setter
    def model_name(self, val):
        assert isinstance(val, str), f"model_name should be a str and is {type(val)}"
        self._model_name = val

    # methods below this point do nothing
    def set_completed_status(self):
        pass

    def set_executing_status(self):
        pass

    def respond(self):
        pass

    def get_current_view(self):
        pass

    def view_completed(self):
        pass
