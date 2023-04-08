import numpy as np
import pytest
from motion_planning_env.cylinder_obstacle import CylinderObstacle
from motion_planning_env.box_obstacle import BoxObstacle
from robot_brain.state import State
from robot_brain.object import Object, UNKNOWN
from robot_brain.global_planning.object_node import ObjectNode
from robot_brain.global_planning.hgraph.point_robot_vel_hgraph import PointRobotVelHGraph
from robot_brain.global_planning.hgraph.drive_act_edge import DriveActionEdge
from robot_brain.exceptions import LoopDetectedException, TwoEdgesPointToSameNodeException

from robot_brain.global_planning.kgraph.kgraph import KGraph

from robot_brain.global_planning.node import NODE_COMPLETED, NODE_UNFEASIBLE, NODE_FAILED
from robot_brain.global_planning.hgraph.action_edge import EDGE_PATH_EXISTS, EDGE_PATH_IS_PLANNED, EDGE_HAS_SYSTEM_MODEL
from robot_brain.global_planning.edge import EDGE_COMPLETED, EDGE_EXECUTING, EDGE_FAILED


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

def test_adding_object(box_obstacle):
    kgraph = KGraph()

    box_obj_1 = Object("obj1", State(), box_obstacle, UNKNOWN)
    box_obj_2 = Object("obj2", State(), box_obstacle, UNKNOWN)

    kgraph.add_object(box_obj_1)
    kgraph.add_object(box_obj_2)
    assert len(kgraph.nodes) == 2

    # box already exist, do not add it again
    with pytest.raises(TypeError):
        kgraph.add_object(box_obj_2)
