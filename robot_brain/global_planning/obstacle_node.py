from robot_brain.global_planning.node import Node
from robot_brain.obstacle import Obstacle

class ObstacleNode(Node):
    """
    Object node.
    """

    def __init__(self, iden, name, obstacle):
        Node.__init__(self, iden)
        self.name = name

        # TODO: find a better name/method to keep track of 
        # uncompleted/completed target nodes, Gijs, 15 okt 2022
        self.completed = False
        self.obstacle = obstacle

    def to_string(self):
        return f"ObstacleNode: {self.iden}"

    @property
    def obstacle(self):
        return self._obstacle

    @obstacle.setter
    def obstacle(self, val):
        assert isinstance(val, Obstacle)
        self._obstacle = val
