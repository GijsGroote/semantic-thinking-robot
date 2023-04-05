import pytest

from motion_planning_env.box_objacle import BoxObject

from robot_brain.global_planning.node import Node
from robot_brain.global_planning.hgraph.object_node import ObjectNode
from robot_brain.object import Object
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.global_planning.graph import Graph
from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.hgraph.point_robot_vel_hgraph import PointRobotVelHGraph
from robot_brain.global_planning.hgraph.hgraph import HGraph
from robot_brain.state import State

@pytest.fixture
def hgraph():
    robot = Object(
            "point_robot",
            State(),
            "empty",
        )
    return PointRobotVelHGraph(robot, "env")

box_dict = {
                "movable": False,
                "type": "box",
                "color": [0/255, 255/255, 0/255, 1],
                "position": [0, 0, 0],
                "geometry": {"length": 1, "width": 1, "height": 1},
            }
prop = BoxObject(name="None-Type-Object", content_dict=box_dict)


def test_is_instance(hgraph):

    obj = Object(name="obj",
            state=State(),
            properties=prop)

    obj_node = ObjectNode(iden=2,
            name="P",
            objacle=obj)

    change_of_state_node = ChangeOfStateNode(2, "P", [])
    k_graph = KGraph()

    assert isinstance(obj_node, Node)
    assert isinstance(change_of_state_node, Node)
    assert isinstance(hgraph, HGraph)
    assert isinstance(hgraph, Graph)
    assert isinstance(k_graph, KGraph)
    assert isinstance(k_graph, Graph)

def test_adding_nodes(hgraph):
    node1 = ObjectNode(1, "P", Object("node1", State(), prop))
    node2 = ObjectNode(2, "P", Object("node1", State(), prop))
    node3 = ObjectNode(3, "P", Object("node1", State(), prop))

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

    obj = Object(name="obj",
            state=State(),
            properties=prop)

    obj_node = ObjectNode(iden=2,
            name="P",
            objacle=obj)

    change_of_state_node = ChangeOfStateNode(6, "wutwat", [])
    # allowed
    hgraph.add_target_node(obj_node)
    hgraph.add_start_node(obj_node)
    hgraph.add_node(obj_node)
    hgraph.add_node(obj_node)
    kgraph.add_node(obj_node)
    kgraph.add_node(change_of_state_node)

    with pytest.raises(TypeError):
        hgraph.add_node(change_of_state_node)

    with pytest.raises(TypeError):
        hgraph.add_start_node(change_of_state_node)
