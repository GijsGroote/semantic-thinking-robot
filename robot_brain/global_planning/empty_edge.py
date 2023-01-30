import numpy as np
from robot_brain.global_planning.edge import Edge
from robot_brain.state import State

INITIALISED = "initialised"
COMPLETED = "completed"


class EmptyEdge(Edge):
    """ Empty edge is the links in the hgraph, that does not have any action in the simulation environmnemt. """

    def __init__(self, iden, source, to):
        Edge.__init__(self, iden, source, to, "empty_edge", "controller?")
        self.status = INITIALISED

    def respond(self, state) -> np.ndarray:
        """ respond to the current state. """
        # how does the empty edge ? respond?
        return np.array([0, 0]) 

    def view_completed(self, current_state: State) -> bool:
        """ view is completed if the test push time is over. """
        return True

    def completed(self) -> bool:
        """ returns true if the edge is completed, otherwise false. """
        return True

    def create_log(self) -> dict:
        """ return a dictionary with metrics. """

        return {"status": self.status,
            "verb": self.verb}

    def ready_for_execution(self) -> bool:
        """ checks if all parameters are set to execute this transition. """
        return True

    def increment_current_view(self):
        pass

    def get_current_view(self) -> State:
        pass

    def to_string(self):
        return f"Edge type: {type(self).__name__}<br>Edge identifier: {self.iden}"
