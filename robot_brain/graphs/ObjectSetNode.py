from robot_brain.graphs.Node import Node
from robot_brain.planning.Object import Object

class ObjectSetNode(Node):

    def __init__(self, id, P, objectSet):
        Node.__init__(self, id, P)
        self.is_class = "object_set_node"
        self.objectSet = objectSet

    def toString(self):
        return "ObjectSetNode: {}\n containing: {} objects".format(self.id, len(self.objectSet))

    @property
    def objectSet(self):
        return self._objectSet

    @objectSet.setter
    def objectSet(self, val):
        # assert val is a list with objects
        assert isinstance(val, list)
        for v in val:
            assert isinstance(v, Object)
        self._objectSet = val

