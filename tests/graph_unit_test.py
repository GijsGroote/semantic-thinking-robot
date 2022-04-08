from robot_brain.graphs.Node import Node
from robot_brain.graphs.ObjectSetNode import ObjectSetNode
from robot_brain.graphs.ConfSetNode import ConfSetNode
from robot_brain.graphs.ChangeOfConfSetNode import ChangeOfConfSetNode

from robot_brain.graphs.Graph import Graph
from robot_brain.graphs.KGraph import KGraph
from robot_brain.graphs.HGraph import HGraph


import unittest


class MyTestCase(unittest.TestCase):
    def test_isInstance(self):
        node1 = ObjectSetNode(2, "P", [])
        hgraph = HGraph()
        kgraph = KGraph()

        self.assertIsInstance(node1, Node)
        self.assertIsInstance(hgraph, HGraph)
        self.assertIsInstance(hgraph, Graph)
        self.assertIsInstance(kgraph, KGraph)
        self.assertIsInstance(kgraph, Graph)

    def test_adding_nodes(self):
        node1 = ObjectSetNode(1, "P", [])
        node2 = ObjectSetNode(2, "P", [])
        node3 = ObjectSetNode(3, "P", [])

        hgraph = HGraph()
        kgraph = KGraph()

        hgraph.addNode(node1)
        hgraph.addNode(node2)
        hgraph.addNode(node3)
        kgraph.addNode(node1)
        kgraph.addNode(node2)
        kgraph.addNode(node3)

        self.assertEqual(len(hgraph.nodes), 3)
        self.assertEqual(len(kgraph.nodes), 3)

    def test_allowed_node_types(self):
        hgraph = HGraph()
        kgraph = KGraph()
        # todo: which nodes are allowed as targetnode/ node in which graph??

        # should be allowed
        hgraph.addNode(ConfSetNode(2, "P", []))
        hgraph.addNode(ObjectSetNode(2, "P", []))
        hgraph.addTargetNode(ConfSetNode(2, "P", []))



        hgraph.addTargetNode(ConfSetNode(2, "P", []))

        with self.assertRaises(AssertionError):
            hgraph.addNode(ChangeOfConfSetNode(2, "ja", []))
            hgraph.addTargetNode(ChangeOfConfSetNode(2, "P", []))

        # todo there are additional nodes which are or are not allowed
