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

            group = None
            if node.is_class is "object_set_node":
                group = "Object Set Node"
            elif node.is_class is "conf_set_node":
                group = "Configuration Set Node"
            elif node.is_class is "change_of_conf_set_node":
                group = "Change of Conf Set Node"
            else:
                warnings.warn("node could not be classified")


            net.add_node(node.id,
                         title=group + ": " + str(node.id)
                               + "<br>objects: " + node.toString() + "<br>",
                         label=node.id,
                         group=group)

        # add edges
        for edge in self.edges:

            net.add_edge(edge.source,
                         edge.to,
                         group=edge.verb,
                         label=edge.verb,
                         title='lonely node',
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
