from robot_brain.graphs.Graph import Graph
from pyvis.network import Network
import numpy as np


class HGraph(Graph):

    def __init__(self):
        super().__init__()
        self.target_node = None

    def addNode(self, node):
        # todo: check this node is a valid objectSetNode
        # is the node not already in the list??
        self.nodes.append(node)

    def addTargetNode(self, node):
        # todo: check this node is a valid objectSetNode

        self.addNode(node)
        self.target_node = node

    @property
    def target_node(self):
        return self._target_node

    @target_node.setter
    def target_node(self, val):
        # input sanitization
        self._target_node = val

