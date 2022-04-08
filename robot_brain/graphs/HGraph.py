import warnings

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
        Graph.__init__(self)
        self.is_class = "hgraph"
        self.target_node = None

    def addNode(self, node):
        if node.is_class is "change_of_conf_set_node":
            warnings.warn("ChangeOfConfSetNode is not allowed in HGraph")

        self.nodes.append(node)

    def addTargetNode(self, node):
        if node.is_class is not "conf_set_node":
            warnings.warn("only a ConfSetNode is allowed as target node in HGraph")

        self.addNode(node)  # sure to add the target node to the node list?
        self.target_node = node

    @property
    def target_node(self):
        return self._target_node

    @target_node.setter
    def target_node(self, val):
        # input sanitization
        self._target_node = val

