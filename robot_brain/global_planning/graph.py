from abc import ABC, abstractmethod

from pyvis.network import Network
from robot_brain.global_planning.edge import Edge


class Graph(ABC):
    """
    Graph defining interface for HGraph and KGraph.
    """
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
    def add_node(self, node):
        pass

    @property
    def edges(self):
        return self._edges

    def add_edge(self, edge):

        if not isinstance(edge, Edge):
            raise TypeError("Only an Edge is allowed as edge")

        source_exists = False
        to_exists = False
        for node in self.nodes:
            if edge.source == node.iden:
                source_exists = True
            if edge.to == node.iden:
                to_exists = True

        if not source_exists:
            raise IndexError(f"edge.source identifyer: {edge.source} does not exist")
        if not to_exists:
            raise IndexError(f"edge.to identifyer: {edge.to} does not exist")

        self._edges.append(edge)
