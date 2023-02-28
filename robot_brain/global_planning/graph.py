from abc import ABC, abstractmethod

from robot_brain.global_planning.node import Node
from robot_brain.global_planning.edge import Edge
from robot_brain.global_planning.action_edge import EDGE_FAILED


class Graph(ABC):
    """
    Graph defining interface for HGraph and KGraph.
    """
    def __init__(self):
        self._nodes = []
        self._edges = []

        self.count_point = 0

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

    def get_node(self, iden) -> Node:
        """ return  node by id, raises error if the identifyer does not exist. """
        node_list = [node for node in self.nodes if node.iden == iden]
        if len(node_list) == 0:
            raise IndexError(f"a node with identifyer {iden} does not exist.")
        else:
            return node_list[0]

    def get_edge(self, iden) -> Edge:
        """ return  edge by id, raises error if the identifyer does not exist. """
        edge_list = [edge for edge in self.edges if edge.iden == iden]
        if len(edge_list) == 0:
            raise IndexError(f"a edge with identifyer {iden} does not exist.")
        else:
            return edge_list[0]


    def unique_node_iden(self) -> int:
        """ return a unique identifyer for a node. """
        iden = 0
        existing_idens = []

        for node in self.nodes:
            existing_idens.append(node.iden)

        while iden in existing_idens:
            iden += 1

        return iden

    def unique_edge_iden(self) -> int:
        """ return a unique identifyer for an edge. """
        iden = 0
        existing_idens = []

        for edge in self.edges:
            existing_idens.append(edge.iden)

        while iden in existing_idens:
            iden += 1

        return iden

    def point_toward_nodes(self, node_iden) -> list:
        """ returns a list with node identifiers where an
        non-failed edge points from node_iden to these nodes. """
        print('in the point toward nodes')
        if self.count_point > 100:
            print('visualise that shit')
            self.visualise(save=False)
            raise ValueError
        else:
            self.count_point += 1

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

        raise ValueError(f"node {node_iden} has no non-failing incoming edges")

