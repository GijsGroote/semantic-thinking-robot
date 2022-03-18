from robot_brain.graphs.Node import Node
from robot_brain.graphs.ObjectSetNode import ObjectSetNode
from robot_brain.graphs.ConfSetNode import ConfSetNode
from robot_brain.graphs.ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.Object import Object
from robot_brain.State import State
from robot_brain.Configuration import Configuration


from robot_brain.graphs.Graph import Graph
from robot_brain.graphs.KGraph import KGraph
from robot_brain.graphs.HGraph import HGraph


import unittest


class MyTestCase(unittest.TestCase):
    def test_isInstance(self):
        node1 = ObjectSetNode(2, "P", [])
        graph = Graph()
        hgraph = HGraph()
        kgraph = KGraph()

        self.assertIsInstance(node1, Node)
        self.assertIsInstance(graph, Graph)
        self.assertIsInstance(hgraph, HGraph)
        self.assertIsInstance(hgraph, Graph)
        self.assertIsInstance(kgraph, KGraph)
        # self.assertIsInstance(kgraph, Graph)
