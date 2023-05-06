import pytest

from motion_planning_env.box_obstacle import BoxObstacle

from robot_brain.global_planning.node import Node
from robot_brain.global_planning.obstacle_node import ObstacleNode
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.global_planning.graph import Graph
from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.hgraph.point_robot_vel_hgraph import PointRobotVelHGraph
from robot_brain.global_planning.hgraph.hgraph import HGraph
from robot_brain.state import State
from robot_brain.obstacle import Obstacle

@pytest.fixture
def hgraph():
    robot = Obstacle(
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
prop = BoxObstacle(name="None-Type-Obstacle", content_dict=box_dict)


def test_is_instance(hgraph):

    obst = Obstacle(name="obst",
            state=State(),
            properties=prop)

    obst_node = ObstacleNode(iden=2,
            name="P",
            obstacle=obst)

    change_of_state_node = ChangeOfStateNode(2, "P", [])
    k_graph = KGraph()

    assert isinstance(obst_node, Node)
    assert isinstance(change_of_state_node, Node)
    assert isinstance(hgraph, HGraph)
    assert isinstance(hgraph, Graph)
    assert isinstance(k_graph, KGraph)
    assert isinstance(k_graph, Graph)

def test_adding_nodes(hgraph):
    node1 = ObstacleNode(1, "P", Obstacle("node1", State(), prop))
    node2 = ObstacleNode(2, "P", Obstacle("node1", State(), prop))
    node3 = ObstacleNode(3, "P", Obstacle("node1", State(), prop))

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

    obst = Obstacle(name="obst",
            state=State(),
            properties=prop)

    obst_node = ObstacleNode(iden=2,
            name="P",
            obstacle=obst)

    change_of_state_node = ChangeOfStateNode(6, "wutwat", [])

    # allowed
    hgraph.add_target_node(obst_node)
    hgraph.add_start_node(obst_node)
    hgraph.add_node(obst_node)
    hgraph.add_node(obst_node)
    kgraph.add_node(obst_node)
    kgraph.add_node(change_of_state_node)

    with pytest.raises(TypeError):
        hgraph.add_node(change_of_state_node)

    with pytest.raises(TypeError):
        hgraph.add_start_node(change_of_state_node)
