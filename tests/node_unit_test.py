import numpy as np
import pytest
from robot_brain.global_planning.hgraph.object_node import ObjectNode
from robot_brain.object import Object
from robot_brain.global_planning.kgraph.change_of_state_node import ChangeOfStateNode
from robot_brain.state import State

def test_object_set_node():
    ob1 = Object("tree", State(pos=np.array([0, 1, 2])), "empty")
    obst_node_1 = ObjectNode(1, "P", ob1, "subtask")

    # test of assertionError was thrown when objectSet does not contain only objects
    with pytest.raises(AssertionError):
        ObjectNode(1, "P", State(), "subtask")

    # test of assertionError was thrown when objectSet is not a list
    with pytest.raises(AssertionError):
        ObjectNode(1, "P", 2, "subtask")

# # TODO make this test about the changeofstatenode
# def test_conf_set_node():
#     conf1 = Configuration()
#     conf2 = Configuration()
#     conf_set_node_1 = ConfSetNode(1, "P", [conf1, conf2])
#
#     # test of configuration set has 2 objects
#     assert len(conf_set_node_1.conf_set) == 2
#
#     # test of assertionError was thrown when confSet does not contain only objects
#     with pytest.raises(AssertionError):
#         ObjectNode(1, "P", [conf1, State()])
#
#     # test of assertionError was thrown when confSet is not a list
#     with pytest.raises(AssertionError):
# ObjectNode(1, "P", 2)
#
# def test_change_of_conf_set_node():
#     conf1 = Configuration()
#     conf2 = Configuration()
#     change_of_conf_set_node = ChangeOfStateNode(
#             1, "P", [(conf1, [True, True]), (conf2, [False, True, False])])
#
#     # test of configuration set has 2 changeOfConfSets
#     assert len(change_of_conf_set_node.change_of_conf_set) == 2
#
#     # test of assertionError was thrown when changeOfConfSet does
#     # not contain only (Configuration, [booleans])
#     with pytest.raises(AssertionError):
#         ChangeOfStateNode(1, "P", [State()])
#
#     # test of assertionError was thrown when changeOfConfSet
#     # contains (not a Configuration, [booleans])
#     with pytest.raises(AssertionError):
#         ChangeOfStateNode(1, "P", [(State(), [True, True])])
#
#     # test of assertionError was thrown when changeOfConfSet
#     # contains (Configuration, [not a booleans])
#     with pytest.raises(AssertionError):
#         ChangeOfStateNode(1, "P", [(conf1, [True, 2])])
#
#     # test of assertionError was thrown when changeOfConfSet is not a list
#     with pytest.raises(AssertionError):
#         ObjectNode(1, "P", 2)
