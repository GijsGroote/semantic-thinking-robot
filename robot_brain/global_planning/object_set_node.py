from robot_brain.global_planning.node import Node
from robot_brain.object import Object

class ObjectSetNode(Node):
    """
    Object set node.
    """

    def __init__(self, iden, name, object_set):
        Node.__init__(self, iden)
        self.name = name

        # TODO: find a better name/method to keep track of uncompleted/completed target nodes, Gijs, 15 okt 2022
        self.completed = False
        self.object_set = object_set

    def to_string(self):
        return f"ObjectSetNode: {self.iden}\n containing: {len(self.object_set)} objects"

    @property
    def object_set(self):
        return self._object_set

    @object_set.setter
    def object_set(self, val):
        # assert val is a list with objects
        assert isinstance(val, list)
        for v in val:
            assert isinstance(v, Object)
        self._object_set = val
