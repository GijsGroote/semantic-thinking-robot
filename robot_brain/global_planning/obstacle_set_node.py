from robot_brain.global_planning.node import Node
from robot_brain.obstacle import Obstacle

class ObstacleSetNode(Node):
    """
    Object set node.
    """

    def __init__(self, iden, name, obstacle_set):
        Node.__init__(self, iden)
        self.name = name

        # TODO: find a better name/method to keep track of 
        # uncompleted/completed target nodes, Gijs, 15 okt 2022
        self.completed = False
        self.obstacle_set = obstacle_set

    def to_string(self):
        return f"ObstacleSetNode: {self.iden}\n containing: {len(self.obstacle_set)} obstacles"

    @property
    def obstacle_set(self):
        return self._obstacle_set

    @obstacle_set.setter
    def obstacle_set(self, val):
        # assert val is a list with objects
        assert isinstance(val, list)
        for v in val:
            assert isinstance(v, Obstacle)
        self._obstacle_set = val
