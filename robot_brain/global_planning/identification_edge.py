import numpy as np
from robot_brain.controller.controller import Controller
from robot_brain.global_planning.edge import Edge
from robot_brain.state import State


from robot_brain.global_planning.edge import INITIALISED, COMPLETED, EXECUTING, FAILED

class IdentificationEdge(Edge):
    """ parent class for every system identification edge. """

     # TODO: the controller here makes no sense!
    def __init__(self, iden, source, to, verb, controller, model_for_edge_iden, sys_model_name: str):
        Edge.__init__(self, iden, source, to, verb, controller)
        self.status = INITIALISED
        self.model_for_edge_iden = model_for_edge_iden

        self.counter = 0 # temp fix

    def respond(self) -> np.ndarray:
        """ respond to the current state. """

        self.counter += 1
        # how does the system identifier respond?
        return np.array([0, 0])

    def view_completed(self) -> bool:
        """ view is completed if the test push time is over. """
        return self.counter >= 100

    def completed(self) -> bool:
        """ returns true if the edge is completed, otherwise false. """
        # wait 50 time steps
        return self.counter >= 20

    def increment_current_view(self):
        """"""
        # TODO: implement this
        return

    def get_current_view(self):
        """"""
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
        return self.status == INITIALISED

    def to_string(self):
        return f"Edge type: {type(self).__name__}<br>Edge identifier: {self.iden}<br>Status: {self.status}<br>"\
                f"System model: {self.system_model.name}"

    def set_executing_status(self):
        assert self.status == INITIALISED,\
            f"before setting status to {EXECUTING} the status must"\
            f"be {INITIALISED} and it's {self.status}"

        print(f'edge {self.iden} status is executing')
        self.status = EXECUTING

    def set_completed_status(self):
        assert self.status == EXECUTING,\
            f"before setting status to {COMPLETED} the status"\
            f"must be {EXECUTING} and it's {self.status}"

        print(f'edge {self.iden} status is completed')
        self.status = COMPLETED
