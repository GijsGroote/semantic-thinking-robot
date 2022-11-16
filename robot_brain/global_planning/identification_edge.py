import numpy as np
from robot_brain.controller.controller import Controller
from robot_brain.global_planning.edge import Edge 

INITIALISED = "initialised"
# TEST_INPUT_READY = "test_input_ready"
EXECUTING = "executing"
COMPLETED = "completed"
# FAILED = "failed"


class IdentificationEdge(Edge):
    """ parent class for every system identification edge. """

    def __init__(self, iden, source, to, verb, controller, model_for_edge_iden, path):
        Edge.__init__(self, iden, source, to, verb, controller, path)
        self.status = INITIALISED
        self.model_for_edge_iden = model_for_edge_iden

    def create_log(self) -> dict:
        """ return a dictionary with metrics. """

        # TODO: this is from an actionEdge, not identificatino edge mate
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
        # identification edge is alwasys ready
        return True

    def to_string(self):
        return f"iden: {self.iden}, controller: {self.controller.name}"

    def set_completed_status(self):
        self.status = COMPLETED

    def set_executing_status(self):
        self.status = EXECUTING
