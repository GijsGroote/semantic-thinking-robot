from robot_brain.graphs.Graph import Graph
from robot_brain.graphs.ConfSetNode import ConfSetNode
from robot_brain.graphs.ObjectSetNode import ObjectSetNode
from pyvis.network import Network
import numpy as np


class HGraph(Graph):

    def __init__(self):
        super().__init__()
        self.target_node = None

    def addNode(self, node):
        assert isinstance(node, ObjectSetNode) or isinstance(node, ConfSetNode)
        self.nodes.append(node)

    def addTargetNode(self, node):
        # todo: check this node is a valid objectSetNode

        assert isinstance(node, ConfSetNode)
        self.addNode(node)
        self.target_node = node

    @property
    def target_node(self):
        return self._target_node

    @target_node.setter
    def target_node(self, val):
        # input sanitization
        self._target_node = val

