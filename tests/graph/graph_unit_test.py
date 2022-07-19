from robot_brain.graph.Node import Node
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode

from robot_brain.graph.Graph import Graph
from robot_brain.graph.KGraph import KGraph
from robot_brain.graph.HGraph import HGraph


import unittest


class MyTestCase(unittest.TestCase):
    def test_isInstance(self):
        obj_set_node = ObjectSetNode(2, "P", [])
        conf_set_node = ConfSetNode(2, "P", [])
        change_of_conf_set_node = ChangeOfConfSetNode(2, "P", [])
        hgraph = HGraph()
        kgraph = KGraph()

        self.assertIsInstance(obj_set_node, Node)
        self.assertIsInstance(conf_set_node, Node)
        self.assertIsInstance(change_of_conf_set_node, Node)
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
        # kgraph = KGraph()
        
        conf_set_node = ConfSetNode(3, "P", [])
        obj_set_node = ObjectSetNode(2, "P", [])
        change_of_conf_set_node = ChangeOfConfSetNode(6, "wutwat", [])

        # allowed 
        hgraph.addTargetNode(conf_set_node)

        # not allowed
        with self.assertRaises(TypeError):
            hgraph.addTargetNode(obj_set_node)
        
        with self.assertRaises(TypeError):
            hgraph.addNode(change_of_conf_set_node)

