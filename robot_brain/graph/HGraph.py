import warnings

# import pandas as pd
from robot_brain.graph.Graph import Graph
from robot_brain.graph.Node import Node
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode 
from pyvis.network import Network
import numpy as np
import pyarrow.feather as feather


class HGraph(Graph):

    def __init__(self):
        Graph.__init__(self)
        self.is_class = "hgraph"
        self.target_node = None

    def addNode(self, node):
        if isinstance(node, ChangeOfConfSetNode):
            raise TypeError("ChangeOfConfSetNode's are not allowed in HGraph")

        self.nodes.append(node)

    def addTargetNode(self, node):
        if not isinstance(node, ConfSetNode):
            raise TypeError("Only ConfSetNode is allowed as targetNode")
         
        self.addNode(node)  
        self.target_node = node

    @property
    def target_node(self):
        return self._target_node

    @target_node.setter
    def target_node(self, val):
        # input sanitization
        self._target_node = val
