from abc import ABC, abstractmethod
from robot_brain.global_variables import STARTING_NODE, CURRENT_NODE, TARGET_NODE

class Node(ABC):
    """
    Abstract class for Node
    """
    def __init__(self, id):
        self.id = id  # identifier
        self.type = None

    @abstractmethod
    def toString(self):
        pass

    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, val):
        self._id = val

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, val):
        self._type = val

    def make_regular_node(self):
        self.type = None 

    def make_starting_node(self):
        self.type = STARTING_NODE 

    def make_current_node(self):
        self.type = CURRENT_NODE 

    def make_target_node(self):
        self.type = TARGET_NODE 

# todo: getter and settor for condition set
