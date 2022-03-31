import pandas as pd

from robot_brain.graphs.Graph import Graph
from robot_brain.graphs.Node import Node
from robot_brain.graphs.ConfSetNode import ConfSetNode
from robot_brain.graphs.ObjectSetNode import ObjectSetNode
from pyvis.network import Network
import numpy as np
import pyarrow.feather as feather


class HGraph(Graph):

    def __init__(self):
        super().__init__()
        self.target_node = None

    def addNode(self, node):
        assert isinstance(node, Node) or isinstance(node, ConfSetNode) # this should be a Objectnode not Node
        self.nodes.append(node)

    def addTargetNode(self, node):
        # print(type(nodee))
        # # todo: check this node is a valid objectSetNode
        # print(isinstance(nodee, ObjectSetNode))
        assert isinstance(node, Node)  # this should be ConfsetNote, but somehow stuff is weird
        self.addNode(node)
        self.target_node = node

    @property
    def target_node(self):
        return self._target_node

    @target_node.setter
    def target_node(self, val):
        # input sanitization
        self._target_node = val

