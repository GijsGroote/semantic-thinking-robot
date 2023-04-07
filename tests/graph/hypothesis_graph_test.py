import numpy as np
import pytest
from motion_planning_env.cylinder_obstacle import CylinderObstacle
from motion_planning_env.box_obstacle import BoxObstacle
from robot_brain.state import State
from robot_brain.object import Object
from robot_brain.global_planning.hgraph.object_node import ObjectNode
from robot_brain.global_planning.hgraph.point_robot_vel_hgraph import PointRobotVelHGraph
from robot_brain.global_planning.hgraph.drive_act_edge import DriveActionEdge
from robot_brain.exceptions import LoopDetectedException, TwoEdgesPointToSameNodeException

from robot_brain.global_planning.node import NODE_COMPLETED, NODE_UNFEASIBLE, NODE_FAILED
from robot_brain.global_planning.hgraph.action_edge import EDGE_PATH_EXISTS, EDGE_PATH_IS_PLANNED, EDGE_HAS_SYSTEM_MODEL
from robot_brain.global_planning.edge import EDGE_COMPLETED, EDGE_EXECUTING, EDGE_FAILED

@pytest.fixture(name="hgraph")
def hgraph_fixture():

    return PointRobotVelHGraph(Object("point_robot",
        State(),
        CylinderObstacle(name="simple_cilinder",
            content_dict={
                    "movable": True,
                    "mass": 1000,
                    "type": "cylinder",
                    "color": [0/255, 255/255, 0/255, 1],
                    "position": [-1, -1, 0.5],
                    "geometry": {"radius": 0.5, "height": 0.25},
                    })))


@pytest.fixture(name="box_obstacle")
def box_obstacle_fixture():
    return BoxObstacle(name="None-Type-Object", content_dict={
                "movable": False,
                "type": "box",
                "color": [0/255, 255/255, 0/255, 1],
                "position": [0, 0, 0],
                "geometry": {"length": 1, "width": 1, "height": 1},
            }
        )


def test_initialisation():
    # initialise correctly
    PointRobotVelHGraph(Object("point_robot", State(), CylinderObstacle(name="simple_cilinder", content_dict={
                    "movable": True,
                    "mass": 1000,
                    "type": "cylinder",
                    "color": [0/255, 255/255, 0/255, 1],
                    "position": [-1, -1, 0.5],
                    "geometry": {"radius": 0.5, "height": 0.25},
                    })))

    # initialise incorrectly
    with pytest.raises(AttributeError):
        PointRobotVelHGraph(None)

def test_adding_nodes(hgraph, box_obstacle):

    node1 = ObjectNode(1, "robot_start", hgraph.robot_obj, "subtask1")
    node2 = ObjectNode(2, "node2", Object("node3", State(), box_obstacle), "subtask1")
    node3 = ObjectNode(3, "robot_target", hgraph.robot_obj, "subtask1")

    hgraph.add_node(node1)
    hgraph.add_node(node2)
    hgraph.add_node(node3)

    assert len(hgraph.nodes) == 3

def test_loop_detection(hgraph, box_obstacle):


    node1 = ObjectNode(1, "robot_start", hgraph.robot_obj, "subtask1")
    node2 = ObjectNode(2, "node2", Object("node3", State(), box_obstacle), "subtask1")
    node3 = ObjectNode(3, "robot_target", hgraph.robot_obj, "subtask1")

    hgraph.add_start_node(node1)
    hgraph.add_node(node2)
    hgraph.add_target_node(node3)

    controller1, model_name1 = hgraph.create_drive_controller(node2.iden)
    controller2, model_name2 = hgraph.create_drive_controller(node3.iden)
    controller3, model_name3 = hgraph.create_drive_controller(node1.iden)

    edge1 = DriveActionEdge(iden=hgraph.unique_edge_iden(),
            source=node1.iden,
            to=node2.iden,
            robot_obj=hgraph.robot_obj,
            verb="driving",
            controller=controller1,
            model_name=model_name1,
            subtask_name="subtask1")

    hgraph.add_edge(edge1)

    edge2 = DriveActionEdge(iden=hgraph.unique_edge_iden(),
            source=node2.iden,
            to=node3.iden,
            robot_obj=hgraph.robot_obj,
            verb="driving",
            controller=controller2,
            model_name=model_name2,
            subtask_name="subtask1")

    hgraph.add_edge(edge2)

    edge3 = DriveActionEdge(iden=hgraph.unique_edge_iden(),
            source=node3.iden,
            to=node1.iden,
            robot_obj=hgraph.robot_obj,
            verb="driving",
            controller=controller3,
            model_name=model_name3,
            subtask_name="subtask1")

    # introduce a loop in graph
    with pytest.raises(LoopDetectedException):
        hgraph.add_edge(edge3)
def test_multiple_edges_point_to_a_node(hgraph):

    node1 = ObjectNode(1, "robot_start", hgraph.robot_obj, "subtask1")
    node2 = ObjectNode(2, "robot_target", hgraph.robot_obj, "subtask1")

    hgraph.add_start_node(node1)
    hgraph.add_target_node(node2)

    controller1, model_name1 = hgraph.create_drive_controller(node2.iden)
    controller2, model_name2 = hgraph.create_drive_controller(node2.iden)

    edge1 = DriveActionEdge(iden=hgraph.unique_edge_iden(),
            source=node1.iden,
            to=node2.iden,
            robot_obj=hgraph.robot_obj,
            verb="driving",
            controller=controller1,
            model_name=model_name1,
            subtask_name="subtask1")

    hgraph.add_edge(edge1)

    edge2 = DriveActionEdge(iden=hgraph.unique_edge_iden(),
            source=node1.iden,
            to=node2.iden,
            robot_obj=hgraph.robot_obj,
            verb="driving",
            controller=controller2,
            model_name=model_name2,
            subtask_name="subtask1")

    with pytest.raises(TwoEdgesPointToSameNodeException):
        hgraph.add_edge(edge2)

def test_wrong_current_node(hgraph):

    node1 = ObjectNode(1, "robot_start", hgraph.robot_obj, "subtask1")
    hgraph.add_node(node1)
    hgraph.current_node = node1

    node1.status = NODE_COMPLETED
    with pytest.raises(AssertionError):
        hgraph.current_node = node1

    node1.status = NODE_UNFEASIBLE
    with pytest.raises(AssertionError):
        hgraph.current_node = node1

    node1.status = NODE_FAILED
    with pytest.raises(AssertionError):
        hgraph.current_node = node1


def test_wrong_current_edge(hgraph, box_obstacle):

    node1 = ObjectNode(1, "robot_start", hgraph.robot_obj, "subtask1")
    node2 = ObjectNode(2, "node2", Object("node3", State(), box_obstacle), "subtask1")

    hgraph.add_node(node1)
    hgraph.add_node(node2)

    controller1, model_name1 = hgraph.create_drive_controller(node2.iden)

    edge1 = DriveActionEdge(iden=hgraph.unique_edge_iden(),
            source=node1.iden,
            to=node2.iden,
            robot_obj=hgraph.robot_obj,
            verb="driving",
            controller=controller1,
            model_name=model_name1,
            subtask_name="subtask1")


    edge1.path_estimator = hgraph.create_drive_path_estimator({})
    edge1.path_estimator.search_path(State(), State(pos=np.array([1,1,0])))

    edge1.status = EDGE_PATH_EXISTS
    with pytest.raises(AssertionError):
        hgraph.current_edge = edge1

    edge1.status = EDGE_HAS_SYSTEM_MODEL
    with pytest.raises(AssertionError):
        hgraph.current_edge = edge1

    edge1.motion_planner = hgraph.create_drive_motion_planner({}, edge1.path_estimator)
    (edge1.path, _) = edge1.motion_planner.search_path(State(), State(pos=np.array([1,1,0])))

    edge1.status = EDGE_PATH_IS_PLANNED
    hgraph.current_edge = edge1

    edge1.status = EDGE_EXECUTING
    with pytest.raises(AssertionError):
        hgraph.current_edge = edge1

    edge1.status = EDGE_COMPLETED
    with pytest.raises(AssertionError):
        hgraph.current_edge = edge1

    edge1.status = EDGE_FAILED
    with pytest.raises(AssertionError):
        hgraph.current_edge = edge1
