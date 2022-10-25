from robot_brain.global_planning.node import Node
from robot_brain.state import State

class StateNode(Node):
    """
    State node.

    TODO: this class is a temp fix, for nodes for which the obstacle is still unknown.
    """
    def __init__(self, iden, name, state):
        Node.__init__(self, iden)
        self.name = name
        self.state = state

    def to_string(self):
        return f"StateNode: {self.iden}"

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, val):
        assert isinstance(val, State)
        self._state = val
