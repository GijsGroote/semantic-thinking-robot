from robot_brain.graphs.Graph import Graph
from robot_brain.graphs.Node import Node
from robot_brain.graphs.ConfSetNode import ConfSetNode
from robot_brain.graphs.ObjectSetNode import ObjectSetNode
from pyvis.network import Network
import numpy as np


class HGraph(Graph):

    def __init__(self):
        super().__init__()
        self.target_node = None

    def addNode(self, node):
        assert isinstance(node, Node) or isinstance(node, ConfSetNode) # this should be a Objectnode not Node
        self.nodes.append(node)

    def addTargetNode(self, nodee):
        print(type(nodee))
        # todo: check this node is a valid objectSetNode
        print(isinstance(nodee, ObjectSetNode))
        assert isinstance(nodee, Node)  # this should be ConfsetNote, but somehow stuff is weird
        self.addNode(nodee)
        self.target_node = nodee

    @property
    def target_node(self):
        return self._target_node

    @target_node.setter
    def target_node(self, val):
        # input sanitization
        self._target_node = val

