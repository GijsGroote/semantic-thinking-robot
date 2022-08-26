from robot_brain.graph.node import Node
from robot_brain.planning.configuration import Configuration


class ChangeOfConfSetNode(Node):
    """
    Node indicating what subspace from a configuration can be changed.
    """
    def __init__(self, iden, name, change_of_conf_set):
        Node.__init__(self, iden)
        self.name = name
        self.change_of_conf_set = change_of_conf_set

    def to_string(self):
        return f"changeOfConfSetNode: {self.iden}\n containing"\
                ": {len(self.change_of_conf_set)} changeOfConf"

    @property
    def change_of_conf_set(self):
        return self._change_of_conf_set

    @change_of_conf_set.setter
    def change_of_conf_set(self, val):
        # assert val is a list with (configuration, [booleans])
        # TODO: the configuration should be of equal shape as the list of booleans
        assert isinstance(val, list)
        for v in val:
            assert isinstance(v, tuple)
            assert isinstance(v[0], Configuration)
            assert isinstance(v[1], list)
            for isbool in v[1]:
                assert isinstance(isbool, bool)
        self._change_of_conf_set = val
