from typing import List

from abc import ABC, abstractmethod

from robot_brain.global_planning.node import Node
from robot_brain.global_planning.edge import Edge
from robot_brain.global_planning.object_node import ObjectNode
from robot_brain.global_planning.hgraph.action_edge import EDGE_FAILED

from robot_brain.global_variables import CREATE_SERVER_DASHBOARD


class Graph(ABC):
    """
    Graph defining interface for HGraph and KGraph.
    """
    def __init__(self):
        self._nodes = {}
        self._edges = {}

    @abstractmethod
    def add_node(self, node: ObjectNode):
        """ add node to the dictionary of nodes. """

    def add_edge(self, edge: Edge):
        """ add edge to dictionary of edges. """
        assert isinstance(edge, Edge), f"edge should be Edge and is {type(edge)}"
        assert edge.source in self.nodes, f"edge.source: {edge.source} does not exist"
        assert edge.to in self.nodes, f"edge.to: {edge.to} does not exist"

        self.edges[edge.iden] = edge
        assert self.is_valid_check()

        if CREATE_SERVER_DASHBOARD:
            self.visualise()

    @abstractmethod
    def is_valid_check(self) -> bool:
        """ check if graph is valid. """

    def get_node(self, node_iden: int) -> Node:
        """ return  node by id, raises error if the identifyer does not exist. """
        assert node_iden in self.nodes
        return self.nodes[node_iden]

    def get_edge(self, edge_iden: int) -> Edge:
        """ return  edge by id, raises error if the identifyer does not exist. """
        assert edge_iden in self.edges
        return self.edges[edge_iden]

    def unique_node_iden(self) -> int:
        """ return a unique identifyer for a node. """
        iden = len(self.nodes)
        while iden in self.nodes:
            iden += 1

        return iden

    def unique_edge_iden(self) -> int:
        """ return a unique identifyer for an edge. """
        iden = len(self.edges)
        while iden in self.edges:
            iden += 1

        return iden

    def get_outgoing_edges(self, node_iden: int) -> List[Edge]:
        """ returns all non-failing edges pointing out of node with node_iden. """

        outgoing_edges = []

        for temp_edge in self.edges.values():
            if temp_edge.source == node_iden and temp_edge.status != EDGE_FAILED:
                outgoing_edges.append(temp_edge)

        return outgoing_edges

    def get_incoming_edges(self, node_iden: int) -> List[Edge]:
        """ returns the non-failing edge pointing toward this node. """

        incoming_edges = []

        for temp_edge in self.edges.values():
            if  temp_edge.to == node_iden and temp_edge.status != EDGE_FAILED:
                incoming_edges.append(temp_edge)

        return incoming_edges

    @property
    def nodes(self):
        return self._nodes

    @property
    def edges(self):
        return self._edges

    @abstractmethod
    def visualise(self, save=True):
        pass
