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

    def visualise(self):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """
        net = Network(directed=True)

        # add nodes
        for node in self.nodes:

            group = "" 
            color = ""
            if isinstance(node, ObjectSetNode):
                group = "Object Set Node"
                color = "#8A307F"
            elif isinstance(node, ConfSetNode):
                group = "Configuration Set Node"
                color = "#79A7D3"
            elif isinstance(node, ChangeOfConfSetNode):
                group = "Change of Conf Set Node"
                color = "#6883BC"
            else:
                raise TypeError("could not classify node")

            net.add_node(node.id,
                    title=group + ":<br>" + node.toString() + "<br>",
                    label=node.id,
                    group=group,
                    color = color)

        # add edges
        for edge in self.edges:

            net.add_edge(edge.source,
                    edge.to,
                    group=edge.verb,
                    label=edge.verb,
                    title="edge:<br>" + edge.toString() + "<br>"
                    )

        # if you want to edit cusomize the graph
        # net.show_buttons(filter_=['physics'])

        net.show("delete-this.html")

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
