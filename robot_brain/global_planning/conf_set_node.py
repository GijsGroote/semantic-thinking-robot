from robot_brain.global_planning.node import Node
from robot_brain.configuration import Configuration


class ConfSetNode(Node):
    """
    Configuration Set node.
    """
    def __init__(self, iden, name, conf_set):
        Node.__init__(self, iden)
        self.name = name
        self.conf_set =conf_set

    def to_string(self):
        return f"ConfSetNode: {self.iden}\n containing: {len(self.conf_set)} configurations"

    @property
    def conf_set(self):
        return self._conf_set

    @conf_set.setter
    def conf_set(self, val):
        # assert val is a list with configurations
        assert isinstance(val, list)
        for v in val:
            assert isinstance(v, Configuration)
        self._conf_set = val
