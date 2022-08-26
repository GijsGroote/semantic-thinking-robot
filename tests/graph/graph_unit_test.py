import unittest

from robot_brain.graph.node import Node
from robot_brain.graph.object_set_node import ObjectSetNode
from robot_brain.graph.conf_set_node import ConfSetNode
from robot_brain.graph.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.graph.graph import Graph
from robot_brain.graph.k_graph import KGraph
from robot_brain.graph.h_graph import HGraph


class MyTestCase(unittest.TestCase):
    """
    Test Graph class and associated classes.
    """
    def test_is_instance(self):
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

        hgraph.add_node(node1)
        hgraph.add_node(node2)
        hgraph.add_node(node3)

        kgraph.add_node(node1)
        kgraph.add_node(node2)
        kgraph.add_node(node3)

        self.assertEqual(len(hgraph.nodes), 3)
        self.assertEqual(len(kgraph.nodes), 3)

    def test_allowed_node_types(self):
        hgraph = HGraph()
        kgraph = KGraph()

        conf_set_node = ConfSetNode(3, "P", [])
        obj_set_node = ObjectSetNode(2, "P", [])
        change_of_conf_set_node = ChangeOfConfSetNode(6, "wutwat", [])

        # allowed
        hgraph.add_target_node(conf_set_node)
        hgraph.add_start_node(obj_set_node)
        hgraph.add_node(conf_set_node)
        hgraph.add_node(obj_set_node)
        kgraph.add_node(obj_set_node)
        kgraph.add_node(change_of_conf_set_node)

        # not allowed
        with self.assertRaises(TypeError):
            hgraph.add_target_node(obj_set_node)

        with self.assertRaises(TypeError):
            hgraph.add_target_node(change_of_conf_set_node)

        with self.assertRaises(TypeError):
            hgraph.add_node(change_of_conf_set_node)

        with self.assertRaises(TypeError):
            hgraph.add_start_node(change_of_conf_set_node)

        with self.assertRaises(TypeError):
            hgraph.add_start_node(conf_set_node)

        with self.assertRaises(TypeError):
            kgraph.add_node(conf_set_node)
