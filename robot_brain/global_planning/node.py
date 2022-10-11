from abc import ABC, abstractmethod

class Node(ABC):
    """
    Abstract class for Node
    """
    def __init__(self, iden):
        self.iden = iden  # identifier
        self.type = None

    @abstractmethod
    def to_string(self):
        pass

    @property
    def iden(self):
        return self._iden

    @iden.setter
    def iden(self, val):
        self._iden = val

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, val):
        self._type = val
