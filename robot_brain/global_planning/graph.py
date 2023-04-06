from abc import ABC, abstractmethod

from robot_brain.global_planning.node import Node
from robot_brain.global_planning.edge import Edge
from robot_brain.global_planning.hgraph.action_edge import EDGE_FAILED


class Graph(ABC):
    """
    Graph defining interface for HGraph and KGraph.
    """
    def __init__(self):
        self._nodes = {}
        self._edges = {}

        self.err_counter = 0

    @abstractmethod
    def visualise(self, save=True):
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
        """ add edge to dictionary of edges. """
        assert isinstance(edge, Edge), f"edge should be Edge and is {type(edge)}"
        assert edge.source in self.nodes, f"edge.source: {edge.source} does not exist"
        assert edge.to in self.nodes, f"edge.to: {edge.to} does not exist"

        self.edges[edge.iden] = edge
        assert self.is_valid_check()

    @abstractmethod
    def is_valid_check(self) -> bool:
        """ check if graph is valid. """

    def get_node(self, iden) -> Node:
        """ return  node by id, raises error if the identifyer does not exist. """
        assert iden in self.nodes
        return self.nodes[iden]

    def get_edge(self, iden) -> Edge:
        """ return  edge by id, raises error if the identifyer does not exist. """
        assert iden in self.edges
        return self.edges[iden]

    def unique_node_iden(self) -> int:
        """ return a unique identifyer for a node. """
        iden = 0

        while iden in self.nodes:
            iden += 1

        return iden

    def unique_edge_iden(self) -> int:
        """ return a unique identifyer for an edge. """
        iden = 0

        while iden in self.edges:
            iden += 1

        print(f"return iden {iden}")

        return iden

    def point_toward_nodes(self, node_iden) -> list:
        """ returns a list with node identifiers where an
        non-failed edge points from node_iden to these nodes. """
        # delete this
        self.err_counter += 1
        if self.err_counter > 5000:
            self.visualise(save=False)
            raise ValueError
        # untill here

        assert any(temp_node.iden == node_iden for temp_node in self.nodes), f"a node node identifier {node_iden} does not exist"
        point_toward_list = []
        for edge in self.edges:
            if edge.status != EDGE_FAILED and node_iden == edge.source:
                point_toward_list.append(edge.to)

        return point_toward_list

    def get_outgoing_edges(self, node_iden) -> list:
        """ returns all non-failing edges pointing toward this node. """

        outgoing_edges = []

        for temp_edge in self.edges:
            if temp_edge.status != EDGE_FAILED and temp_edge.source == node_iden:
                outgoing_edges.append(temp_edge)

        return outgoing_edges

    def get_incoming_edge(self, node_iden) -> Edge:
        """ returns the non-failing edge pointing toward this node. """

        for temp_edge in self.edges:
            if temp_edge.status != EDGE_FAILED and temp_edge.to == node_iden:
                return temp_edge


