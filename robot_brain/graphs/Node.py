from abc import ABC, abstractmethod


class Node(ABC):
    """
    Abstract class for Node
    """

    def __init__(self, id, P):
        self.id = id  # identifier
        self.P = P  # todo, create condition set

    @abstractmethod
    def toString(self):
        pass
