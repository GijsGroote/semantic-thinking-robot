from abc import ABC, abstractmethod
from robot_brain.graphs.Edge import Edge
from pyvis.network import Network


class Graph(ABC):

    def __init__(self):
        self._nodes = []
        self._edges = []

    def save_as_html(self):
        # make this function such that it updates if it is already present
        net = Network()

        # add nodes
        for node in self.nodes:
            net.add_node(node.id, label=node.id)

        # add edges
        for edge in self.edges:
            net.add_edge(edge.source, edge.to)

        net.show("name.html")

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
        assert isinstance(edge, Edge)
        self._edges.append(edge)

