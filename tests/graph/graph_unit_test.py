from robot_brain.global_planning.node import Node
from robot_brain.global_planning.object_set_node import ObjectSetNode
from robot_brain.global_planning.conf_set_node import ConfSetNode
from robot_brain.global_planning.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.global_planning.graph import Graph
from robot_brain.global_planning.kgraph.kgraph import KGraph

from robot_brain.global_planning.hgraph.boxer_robot_hgraph import BoxerRobotHGraph
from robot_brain.global_planning.hgraph.hgraph import HGraph

from robot_brain.state import State
from robot_brain.object import Object
import pytest


@pytest.fixture
def hgraph():
    robot = Object(
            "point_robot",
            State(),
            "urdf",
        )
    return BoxerRobotHGraph(robot)

def test_is_instance(hgraph):
    obj_set_node = ObjectSetNode(2, "P", [])
    conf_set_node = ConfSetNode(2, "P", [])
    change_of_conf_set_node = ChangeOfConfSetNode(2, "P", [])
    kgraph = KGraph()

    assert isinstance(obj_set_node, Node)
    assert isinstance(conf_set_node, Node)
    assert isinstance(change_of_conf_set_node, Node)
    assert isinstance(hgraph, HGraph)
    assert isinstance(hgraph, Graph)
    assert isinstance(kgraph, KGraph)
    assert isinstance(kgraph, Graph)

def test_adding_nodes(hgraph):
    node1 = ObjectSetNode(1, "P", [])
    node2 = ObjectSetNode(2, "P", [])
    node3 = ObjectSetNode(3, "P", [])

    kgraph = KGraph()

    hgraph.add_node(node1)
    hgraph.add_node(node2)
    hgraph.add_node(node3)

    kgraph.add_node(node1)
    kgraph.add_node(node2)
    kgraph.add_node(node3)

    assert len(hgraph.nodes) == 3
    assert len(kgraph.nodes)== 3

def test_allowed_node_types(hgraph):
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
    # TODO: this test is an exeption, Gijs Still has to decide if it is time for configurations or only states
    # with pytest.raises(TypeError):
    #     hgraph.add_target_node(obj_set_node)
    #
    with pytest.raises(TypeError):
        hgraph.add_target_node(change_of_conf_set_node)

    with pytest.raises(TypeError):
        hgraph.add_node(change_of_conf_set_node)

    with pytest.raises(TypeError):
        hgraph.add_start_node(change_of_conf_set_node)

    with pytest.raises(TypeError):
        hgraph.add_start_node(conf_set_node)

    with pytest.raises(TypeError):
        kgraph.add_node(conf_set_node)
