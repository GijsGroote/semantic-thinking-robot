from robot_brain.global_planning.node import Node
from robot_brain.state import State


class ChangeOfStateNode(Node):
    """
    Node indicating what subspace from a state can be changed.
    """
    def __init__(self, iden, name, change_of_state):
        Node.__init__(self, iden)
        self.name = name
        self.change_of_state = change_of_state

    def to_string(self):
        return f"changeOfConfSetNode: {self.iden}\n containing"\
                ": {len(self.change_of_state)} changeOfConf"

    @property
    def change_of_state(self):
        return self._change_of_state

    @change_of_state.setter
    def change_of_state(self, val):
        # assert val is a list with (configuration, [booleans])
        # TODO: the configuration should be of equal shape as the list of booleans
        assert isinstance(val, list)
        for v in val:
            assert isinstance(v, tuple)
            assert isinstance(v[0], State)
            assert isinstance(v[1], list)
            for isbool in v[1]:
                assert isinstance(isbool, bool)
        self._change_of_state = val
