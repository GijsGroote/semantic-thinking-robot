import numpy as np
from robot_brain.controller.controller import Controller
from robot_brain.global_planning.edge import Edge

INITIALISED = "initialised"
PATH_EXISTS = "path_exists"
HAS_SYSTEM_MODEL = "has_system_model"
PATH_IS_PLANNED = "path_is_planned"
EXECUTING = "executing"
COMPLETED = "completed"
FAILED = "failed"


class ActionEdge(Edge):
    """ Parent class for all actions. """

    def __init__(self, iden, source, to, verb, controller, path):
        Edge.__init__(self, iden, source, to, verb, controller, path)
        self.status = INITIALISED

    def create_log(self) -> dict:
        """ return a dictionary with metrics. """
        # TODO: maybe you want to be less strict here on that prediction error, gijs 11 nov 2022
        pred_error = self.controller.pred_error
        assert isinstance(pred_error, list), f"pred_error should be a list and is {type(pred_error)}"
        assert len(pred_error) > 0, f"pred_error should contain at least 1 value and containts {len(pred_error)} values"
        assert all(isinstance(err, float) for err in pred_error), "pred_error should contain only floats but does not"

        # TODO: this should be done differnetly, due to fist predition error begin 0 mostly
        if len(pred_error) != 1:
            min_pred_error = pred_error[0:-1]
        else:
            min_pred_error = pred_error

        return {
                    # status can be: initialised, sys_model, path_planned, failed or completed
                    "status": self.status,
                    "verb": self.verb,
                    "controller_type": self.controller.name,
                    "avg_pred_error": np.round(np.average(pred_error), decimals=5),
                    "max_pred_error": np.round(max(pred_error), decimals=5),
                    "min_pred_error": np.round(min(min_pred_error), decimals=5),
                    "final_pose_error": None,
                }

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
        return f"iden: {self.iden}, controller: {self.controller.name}"

    def set_path_exist_status(self):
        assert self.status == INITIALISED, f"before setting status to {PATH_EXISTS} the status must be {INITIALISED} and it's {self.status}"
        self.status = PATH_EXISTS

    def set_has_system_model_status(self):
        assert self.status == PATH_EXISTS, f"before setting status to {HAS_SYSTEM_MODEL} the status must be {PATH_EXISTS} and it's {self.status}"
        self.status = HAS_SYSTEM_MODEL 

    def set_path_is_planned_status(self):
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

