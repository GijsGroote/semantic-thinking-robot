import numpy as np
from robot_brain.global_planning.edge import Edge

from robot_brain.controller.controller import Controller

from robot_brain.object import Object

from robot_brain.global_planning.edge import EDGE_INITIALISED, EDGE_COMPLETED, EDGE_EXECUTING, EDGE_FAILED

class IdentificationEdge(Edge):
    """ parent class for every system identification edge. """

    def __init__(self, iden: int,
            source: int,
            to: int,
            verb: str,
            controller: Controller,
            subtask_name: str,
            model_for_edge_iden: int):

        Edge.__init__(self, iden, source, to, verb, controller, subtask_name)
        self.status = EDGE_INITIALISED
        self.model_for_edge_iden = model_for_edge_iden

    def respond(self) -> np.ndarray:
        """ respond to the current state. """

        return np.array([0, 0])

    def view_completed(self) -> bool:
        """ view is completed if the test push time is over. """
        return True # the identification edge is always ready mate

    def completed(self) -> bool:
        """ returns true if the edge is completed, otherwise false. """
        return True

    def increment_current_view(self):
        """ empty docstring """
        # TODO: implement this
        return

    def get_current_view(self):
        """ emtpy docstring """
        # TODO: implement this
        return

    def create_log(self) -> dict:
        """ return a dictionary with metrics. """

        return {
            # status can be: initialised, sys_model, path_planned, failed or completed
            "status": self.status,
            "verb": self.verb,
               }

    def ready_for_execution(self) -> bool:
        """ checks if all parameters are set to execute this transition. """
        return self.status in [EDGE_INITIALISED, EDGE_EXECUTING]

    def to_string(self):
        return f"Edge type: {type(self).__name__}<br>Edge identifier: {self.iden}<br>Status: {self.status}<br>"\
                f"System model: {self.system_model.name}"

    def set_executing_status(self):
        assert self.status in [EDGE_INITIALISED, EDGE_EXECUTING],\
            f"before setting status to {EDGE_EXECUTING} the status must"\
            f"be {EDGE_INITIALISED} or {EDGE_EXECUTING} and it's {self.status}"

        self.status = EDGE_EXECUTING

    def set_completed_status(self):
        assert self.status == EDGE_EXECUTING,\
            f"before setting status to {EDGE_COMPLETED} the status "\
            f"must be {EDGE_EXECUTING} and it is: {self.status}"

        self.status = EDGE_COMPLETED

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, val):
        assert val in [EDGE_INITIALISED, EDGE_COMPLETED,
                EDGE_EXECUTING, EDGE_FAILED], f"invalid status: {val}"
        self._status = val

    @property
    def model_for_edge_iden(self):
        return self._model_for_edge_iden

    @model_for_edge_iden.setter
    def model_for_edge_iden(self, val):
        assert isinstance(val, int), f"model_for_edge_iden should be int and is: {type(val)}"
        self._model_for_edge_iden = val
