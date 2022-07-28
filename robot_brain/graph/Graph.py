import warnings
from abc import ABC, abstractmethod
from robot_brain.graph.Edge import Edge

from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.Node import Node

from pyvis.network import Network


class Graph(ABC):

    def __init__(self):
        self._nodes = []
        self._edges = []

    @abstractmethod
    def visualise(self):
        pass

    @property
    def nodes(self):
        return self._nodes

    @abstractmethod
    def addNode(self, val):
        pass

    @property
    def edges(self):
        return self._edges

    def addEdge(self, edge):
        # todo: input sanitizition:
        if not isinstance(edge, Edge):
            raise TypeError("Only an Edge is allowed as edge")
            
        self._edges.append(edge)
