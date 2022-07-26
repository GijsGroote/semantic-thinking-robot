from abc import ABC, abstractmethod

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

