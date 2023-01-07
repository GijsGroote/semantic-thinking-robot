from robot_brain.global_planning.node import Node
from robot_brain.obstacle import Obstacle

INITIALISED = "initialised"
COMPLETED = "completed"
UNFEASIBLE = "unfeasible"

class ObstacleNode(Node):
    """
    Object node.
    """

    def __init__(self, iden, name, obstacle, subtask_name=None):
        Node.__init__(self, iden)
        self.name = name
        self.status = INITIALISED
        self.obstacle = obstacle
        self.subtask_name = subtask_name

    def to_string(self):
        return f"Node identifier: {self.iden}<br>Status: {self.status}<br>In subtask: {self.subtask_name}"\
                f"<br>With Obstacle: {self.obstacle.name}"

    def ready_for_execution(self) -> bool:
        """ returns true if the node is ready for execution. """
        return self.status == INITIALISED

    @property
    def obstacle(self):
        return self._obstacle

    @obstacle.setter
    def obstacle(self, val):
        assert isinstance(val, Obstacle), f"obstacle should be an Obstacle and is {type(val)}"
        self._obstacle = val
