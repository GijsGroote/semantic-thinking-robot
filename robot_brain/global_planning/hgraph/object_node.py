from robot_brain.global_planning.node import Node
from robot_brain.object import Object


class ObjectNode(Node):
    """
    Object node.
    """

    def __init__(self, iden, name, obj, subtask_name=None):
        Node.__init__(self, iden)
        self.name = name
        self.obj = obj
        self.subtask_name = subtask_name

    def to_string(self):
        return f"Node identifier: {self.iden}<br>Status: {self.status}<br>In subtask: {self.subtask_name}"\
                f"<br>With Object: {self.obj.name}<br>2d pos = {self.obj.state.get_2d_pose()}<br>type = {self.obj.type}"

    @property
    def obj(self):
        return self._obj

    @obj.setter
    def obj(self, val):
        assert isinstance(val, Object), f"object should be an Object and is {type(val)}"
        self._obj = val
