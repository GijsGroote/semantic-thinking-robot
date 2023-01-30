import numpy as np
from abc import abstractmethod
from robot_brain.controller.controller import Controller
from robot_brain.global_planning.edge import Edge
from robot_brain.global_planning.hgraph.local_planning.sample_based.motion_planner import MotionPlanner
from robot_brain.global_planning.hgraph.local_planning.graph_based.path_estimator import PathEstimator

from robot_brain.state import State

from robot_brain.global_planning.edge import INITIALISED, COMPLETED, EXECUTING, FAILED
PATH_EXISTS = "path_exists"
HAS_SYSTEM_MODEL = "has_system_model"
PATH_IS_PLANNED = "path_is_planned"

class ActionEdge(Edge):
    """ Parent class for all actions. """

    def __init__(self, iden, source, to, verb, controller):
        Edge.__init__(self, iden, source, to, verb, controller)
        self.status = INITIALISED

        self._motion_planner = None
        self._path_estimator = None

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
                pos=np.array([self.motion_planner.shortest_path[self.path_pointer][0], self.motion_planner.shortest_path[self.path_pointer][1], 0]),
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
            "status": self.status,
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
        """ checks if all parameters are set to execute this transition. """
        if self.status == PATH_IS_PLANNED:
            assert isinstance(self.controller, Controller),\
                    f"controller is not of type Controller but {type(self.controller)}"
            assert isinstance(self.motion_planner.shortest_path, list),\
                    f"path should be a list and is {type(self.motion_planner.shortest_path)}"
            assert len(self.motion_planner.shortest_path) >= 2,\
                    f"path should contain more then 2 positions and contains {len(self.motion_planner.shortest_path)}"

            return True
        else:
            return False

    def to_string(self):
        return f"Edge type: {type(self).__name__}<br>Edge identifier: {self.iden}<br>Status: {self.status}<br>"\
                f"Controller: {self.controller.name}<br>System model: {self.system_model.name}"

    def set_path_exist_status(self):
        assert isinstance(self.path_estimator, PathEstimator),\
                f"path_estimator should be PathEstimator and is {type(self.path_estimator)}"
        assert isinstance(self.path_estimation, list),\
                f"path_estimation should be list and is {type(self.path_estimation)}"
        assert self.status == INITIALISED,\
                f"before setting status to {PATH_EXISTS} the status must be {INITIALISED} and it's {self.status}"

        self.status = PATH_EXISTS

    def set_has_system_model_status(self):
        assert self.status == PATH_EXISTS,\
                f"before setting status to {HAS_SYSTEM_MODEL} the status must be {PATH_EXISTS} and it's {self.status}"
        self.status = HAS_SYSTEM_MODEL

    def set_path_is_planned_status(self):
        assert isinstance(self.motion_planner, MotionPlanner),\
        f"motion_planner should be MotionPlanner and is {type(self.motion_planner)}"
        assert isinstance(self.motion_planner.shortest_path, list),\
        f"motion_planner should be list and is {type(self.motion_planner)}"
        assert self.status == HAS_SYSTEM_MODEL,\
                f"before setting status to {PATH_IS_PLANNED} the status"\
                f"must be {HAS_SYSTEM_MODEL} and it's {self.status}"

        self.status = PATH_IS_PLANNED

    def set_executing_status(self):
        assert self.status == PATH_IS_PLANNED,\
        f"before setting status to {EXECUTING} the status must"\
        f"be {PATH_IS_PLANNED} and it's {self.status}"
        self.status = EXECUTING

    def set_completed_status(self):
        assert self.status == EXECUTING,\
        f"before setting status to {COMPLETED} the status"\
        f"must be {EXECUTING} and it's {self.status}"
        self.status = COMPLETED

    def set_failed_status(self):
        """ from any status the status can be come FAILED. """
        self.status = FAILED

    @property
    def path_estimator(self):
        return self._path_estimator

    @path_estimator.setter
    def path_estimator(self, p_e):
        assert isinstance(p_e, PathEstimator),\
        f"path estimator should be of type PathEstimator, but is {type(p_e)}"
        self._path_estimator = p_e

    @property
    def motion_planner(self):
        return self._motion_planner

    @motion_planner.setter
    def motion_planner(self, m_p):
        assert isinstance(m_p, MotionPlanner),\
        f"path estimator should be of type MotionPlanner, but is {type(m_p)}"
        self._motion_planner = m_p
