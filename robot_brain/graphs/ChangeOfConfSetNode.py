from robot_brain.graphs.Node import Node
from robot_brain.Configuration import Configuration


class ChangeOfConfSetNode(Node):

    def __init__(self, id, P, changeOfConfSet):
        super().__init__(id, P)
        self.changeOfConfSet = changeOfConfSet

    def toString(self):
        return "changeOfConfSetNode: {}\n containing: {} changeOfConf".format(self.id, len(self.changeOfConfSet))

    @property
    def changeOfConfSet(self):
        return self._changeOfConfSet

    @changeOfConfSet.setter
    def changeOfConfSet(self, val):
        # assert val is a list with (configuration, [booleans])
        # todo: the configuration should be of equal shape as the list of booleans
        assert isinstance(val, list)
        for v in val:
            assert isinstance(v, tuple)
            assert isinstance(v[0], Configuration)
            assert isinstance(v[1], list)
            for isbool in v[1]:
                assert isinstance(isbool, bool)
        self._changeOfConfSet = val
