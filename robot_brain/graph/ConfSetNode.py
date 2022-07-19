from robot_brain.graph.Node import Node
from robot_brain.planning.Configuration import Configuration


class ConfSetNode(Node):

    def __init__(self, id, P, confSet):
        Node.__init__(self, id, P)
        self.is_class = "conf_set_node"
        self.confSet = confSet

    def toString(self):
        return "ConfSetNode: {}\n containing: {} configurations".format(self.id, len(self.confSet))

    @property
    def confSet(self):
        return self._confSet

    @confSet.setter
    def confSet(self, val):
        # assert val is a list with configurations
        assert isinstance(val, list)
        for v in val:
            assert isinstance(v, Configuration)
        self._confSet = val
