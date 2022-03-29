from robot_brain.graphs.ObjectSetNode import ObjectSetNode
from robot_brain.graphs.ConfSetNode import ConfSetNode
from robot_brain.graphs.ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.planning.Object import Object
from robot_brain.planning.State import State
from robot_brain.planning.Configuration import Configuration

import numpy as np


import unittest


class MyTestCase(unittest.TestCase):


    def test_object_set_node(self):
        ob1 = Object("tree", State(pos=np.array([0, 1, 2])), "urdf")
        ob2 = Object("bush", State(), "urdf")
        objSetNode1 = ObjectSetNode(1, "P", [ob1, ob2])

        # test of object set has 2 objects
        self.assertEqual(len(objSetNode1.objectSet), 2)

        # test of assertionError was thrown when objectSet does not contain only objects
        with self.assertRaises(AssertionError):
            ObjectSetNode(1, "P", [ob1, State()])

        # test of assertionError was thrown when objectSet is not a list
        with self.assertRaises(AssertionError):
            ObjectSetNode(1, "P", 2)

    def test_conf_set_node(self):
        conf1 = Configuration()
        conf2 = Configuration()
        confSetNode1 = ConfSetNode(1, "P", [conf1, conf2])

        # test of configuration set has 2 objects
        self.assertEqual(len(confSetNode1.confSet), 2)

        # test of assertionError was thrown when confSet does not contain only objects
        with self.assertRaises(AssertionError):
            ObjectSetNode(1, "P", [conf1, State()])

        # test of assertionError was thrown when confSet is not a list
        with self.assertRaises(AssertionError):
            ObjectSetNode(1, "P", 2)

    def test_change_of_conf_set_node(self):
        conf1 = Configuration()
        conf2 = Configuration()
        changeOfConfSetNode = ChangeOfConfSetNode(1, "P", [(conf1, [True, True]), (conf2, [False, True, False])])

        # test of configuration set has 2 changeOfConfSets
        self.assertEqual(len(changeOfConfSetNode.changeOfConfSet), 2)

        # test of assertionError was thrown when changeOfConfSet does not contain only (Configuration, [booleans])
        with self.assertRaises(AssertionError):
            ChangeOfConfSetNode(1, "P", [State()])

        # test of assertionError was thrown when changeOfConfSet contains (not a Configuration, [booleans])
        with self.assertRaises(AssertionError):
            ChangeOfConfSetNode(1, "P", [(State(), [True, True])])

        # test of assertionError was thrown when changeOfConfSet contains (Configuration, [not a booleans])
        with self.assertRaises(AssertionError):
            ChangeOfConfSetNode(1, "P", [(conf1, [True, 2])])

        # test of assertionError was thrown when changeOfConfSet is not a list
        with self.assertRaises(AssertionError):
            ObjectSetNode(1, "P", 2)


        # graph = Graph
        #
        # # add 3 nodes
        # graph.addNode(Node(4))
        # graph.addNode(Node(2))
        # graph.addNode(Node(1))
        #
        # self.assertEqual(len(graph.getNodes()), 3)




if __name__ == '__main__':
    unittest.main()
