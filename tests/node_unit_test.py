import unittest
import numpy as np

from robot_brain.graph.object_set_node import ObjectSetNode
from robot_brain.graph.conf_set_node import ConfSetNode
from robot_brain.graph.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.planning.object import Object
from robot_brain.planning.state import State
from robot_brain.planning.configuration import Configuration


class MyTestCase(unittest.TestCase):
    """
    Unittest the Graph class and subclasses.
    """

    def test_object_set_node(self):
        ob1 = Object("tree", State(pos=np.array([0, 1, 2])), "urdf")
        ob2 = Object("bush", State(), "urdf")
        obj_set_node_1 = ObjectSetNode(1, "P", [ob1, ob2])

        # test of object set has 2 objects
        self.assertEqual(len(obj_set_node_1.object_set), 2)

        # test of assertionError was thrown when objectSet does not contain only objects
        with self.assertRaises(AssertionError):
            ObjectSetNode(1, "P", [ob1, State()])

        # test of assertionError was thrown when objectSet is not a list
        with self.assertRaises(AssertionError):
            ObjectSetNode(1, "P", 2)

    def test_conf_set_node(self):
        conf1 = Configuration()
        conf2 = Configuration()
        conf_set_node_1 = ConfSetNode(1, "P", [conf1, conf2])

        # test of configuration set has 2 objects
        self.assertEqual(len(conf_set_node_1.conf_set), 2)

        # test of assertionError was thrown when confSet does not contain only objects
        with self.assertRaises(AssertionError):
            ObjectSetNode(1, "P", [conf1, State()])

        # test of assertionError was thrown when confSet is not a list
        with self.assertRaises(AssertionError):
            ObjectSetNode(1, "P", 2)

    def test_change_of_conf_set_node(self):
        conf1 = Configuration()
        conf2 = Configuration()
        change_of_conf_set_node = ChangeOfConfSetNode(
                1, "P", [(conf1, [True, True]), (conf2, [False, True, False])])

        # test of configuration set has 2 changeOfConfSets
        self.assertEqual(len(change_of_conf_set_node.change_of_conf_set), 2)

        # test of assertionError was thrown when changeOfConfSet does
        # not contain only (Configuration, [booleans])
        with self.assertRaises(AssertionError):
            ChangeOfConfSetNode(1, "P", [State()])

        # test of assertionError was thrown when changeOfConfSet
        # contains (not a Configuration, [booleans])
        with self.assertRaises(AssertionError):
            ChangeOfConfSetNode(1, "P", [(State(), [True, True])])

        # test of assertionError was thrown when changeOfConfSet
        # contains (Configuration, [not a booleans])
        with self.assertRaises(AssertionError):
            ChangeOfConfSetNode(1, "P", [(conf1, [True, 2])])

        # test of assertionError was thrown when changeOfConfSet is not a list
        with self.assertRaises(AssertionError):
            ObjectSetNode(1, "P", 2)


if __name__ == '__main__':
    unittest.main()
