import numpy as np
from robot_brain.controller.controller import Controller
from robot_brain.global_planning.edge import Edge
from robot_brain.global_planning.hgraph.local_planning.sample_based.motion_planner import MotionPlanner
from robot_brain.global_planning.hgraph.local_planning.graph_based.configuration_grid_map import ConfigurationGridMap
from robot_brain.dynamics import Dynamics 
from robot_brain.state import State

INITIALISED = "initialised"
PATH_EXISTS = "path_exists"
HAS_SYSTEM_MODEL = "has_system_model"
PATH_IS_PLANNED = "path_is_planned"
EXECUTING = "executing"
COMPLETED = "completed"
FAILED = "failed"


class ActionEdge(Edge):
    """ Parent class for all actions. """

    def __init__(self, iden, source, to, verb, controller):
        Edge.__init__(self, iden, source, to, verb, controller)
        self.status = INITIALISED

        self._motion_planner = None
        self._path_estimator = None

    def respond(self, state, obst_state= None) -> np.ndarray:
        """ respond to the current state. """
        if obst_state is None:
            return self.controller.respond(state)
        else:
            return self.controller.respond(state, obst_state=obst_state)

    def view_completed(self, current_state: State) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """
        if np.linalg.norm(current_state.get_xy_position() - self.get_current_target().get_xy_position()) < 0.5:
            return True
        else:
            return False

    def completed(self) -> bool:
        """ returns true if the edge is completed, otherwise false. """
        return self.path_pointer >= len(self.path)-1

    def increment_current_target(self):
        """ updates toward the next current target from path. """

        if self.path_pointer < len(self.path)-1:
            self.path_pointer += 1

        if  len(self.path[self.path_pointer]) == 3:
            orien = self.path[self.path_pointer][2]
        else:
            orien = 0

        next_target = State(
                pos=np.array([self.path[self.path_pointer][0], self.path[self.path_pointer][1], 0]),
                ang_p=np.array([0, 0, orien])
                )

        self.controller.set_target_state(next_target)

    def get_current_target(self) -> State:
        """ returns the current target the controller tries to steer toward. """
        if len(self.path[self.path_pointer]) == 3:
            orien = self.path[self.path_pointer][2]
        else:
            orien = 0
        return State(pos=np.append(self.path[self.path_pointer][0:2], [[0]]),
                ang_p=np.array([0, 0, orien]))

    def create_log(self) -> dict:
        """ return a dictionary with metrics. """
        # TODO: maybe you want to be less strict here on that prediction error, gijs 11 nov 2022
        pred_error = self.controller.pred_error
        assert isinstance(pred_error, list), f"pred_error should be a list and is {type(pred_error)}"
        assert all(isinstance(err, float) for err in pred_error), "pred_error should contain only floats but does not"

        log = {
            "status": self.status,
            "verb": self.verb,
            "controller_type": self.controller.name,
            "final_pose_error": None,
            }

        # TODO: this should be done differnetly, due to fist predition error begin 0 mostly
        if len(pred_error) > 1:
            log["avg_pred_error"] = np.round(np.average(pred_error), decimals=5),
            log["max_pred_error"] = np.round(max(pred_error), decimals=5),
            log["min_pred_error"] = np.round(min(pred_error[0:-1]), decimals=5),

        return log

    def ready_for_execution(self) -> bool:
        """ checks if all parameters are set to execute this transition. """
        if self.status == PATH_IS_PLANNED:
            assert isinstance(self.controller, Controller), f"controller is not of type Controller but {type(self.controller)}"
            assert self.path != False, f"path is False"
            # TODO: Are there additional assertions to check?
            return True
        else:
            return False

    def to_string(self):
        return f"iden: {self.iden}<br>status:{self.status}, controller: {self.controller.name}"

    def set_path_exist_status(self):
        assert isinstance(self.path_estimator, ConfigurationGridMap), f"path_estimator should be ConfigurationGridMap and is {type(self.path_estimator)}"
        assert isinstance(self.path_estimation, list), f"path_estimation should be list and is {type(self.path_estimation)}"
        assert self.status == INITIALISED, f"before setting status to {PATH_EXISTS} the status must be {INITIALISED} and it's {self.status}"
        self.status = PATH_EXISTS

    def set_has_system_model_status(self):
        # assert isinstance(self.dyn_model, Dynamics), "dyn_model should be of type Dynamics and is {type(self.dyn_model)}"
        assert self.status == PATH_EXISTS, f"before setting status to {HAS_SYSTEM_MODEL} the status must be {PATH_EXISTS} and it's {self.status}"
        self.status = HAS_SYSTEM_MODEL 

    def set_path_is_planned_status(self):
        assert isinstance(self.motion_planner, MotionPlanner), f"motion_planner should be MotionPlanner and is {type(self.motion_planner)}"
        assert isinstance(self.path, list), f"motion_planner should be list and is {type(self.motion_planner)}"
        assert self.status == HAS_SYSTEM_MODEL, f"before setting status to {PATH_IS_PLANNED} the status must be {HAS_SYSTEM_MODEL} and it's {self.status}"
        self.status = PATH_IS_PLANNED

    def set_executing_status(self):
        assert self.status == PATH_IS_PLANNED, f"before setting status to {EXECUTING} the status must be {PATH_IS_PLANNED} and it's {self.status}"
        self.status = EXECUTING

    def set_completed_status(self):
        assert self.status == EXECUTING, f"before setting status to {COMPLETED} the status must be {EXECUTING} and it's {self.status}"
        self.status = COMPLETED

    def set_failed_status(self):
        """ from any status the status can be come FAILED. """
        self.status = FAILED

    @property
    def path_estimator(self):
        return self._path_estimator

    @path_estimator.setter
    def path_estimator(self, pe):
        assert isinstance(pe, ConfigurationGridMap), f"path estimator should be of type ConfigurationGridMap, but is {type(pe)}"
        self._path_estimator = pe

    @property
    def motion_planner(self):
        return self._motion_planner

    @motion_planner.setter
    def motion_planner(self, mp):
        assert isinstance(mp, MotionPlanner), f"path estimator should be of type MotionPlanner, but is {type(mp)}"
        self._motion_planner = mp 
