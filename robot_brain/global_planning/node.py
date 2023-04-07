from abc import ABC, abstractmethod

NODE_INITIALISED = "node:initialised"
NODE_COMPLETED = "node:completed"
NODE_UNFEASIBLE = "node:unfeasible"
NODE_FAILED = "node:failed"

class Node(ABC):
    """
    Abstract class for Node
    """
    def __init__(self, iden):
        self.iden = iden  # identifier
        self.type = None
        self.status = NODE_INITIALISED

    def ready_for_execution(self) -> bool:
        """ returns true if the node is ready for execution. """
        return self.status == NODE_INITIALISED

    @abstractmethod
    def to_string(self):
        """ create a human readable format of this object. """

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
