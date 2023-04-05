
import numpy as np
import pytest

from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.global_planning.hgraph.object_node import ObjectNode
from robot_brain.object import Object, FREE, MOVABLE, UNKNOWN, UNMOVABLE
from robot_brain.state import State
from robot_brain.global_planning.hgraph.point_robot_vel_hgraph import PointRobotVelHGraph
from robot_brain.global_variables import POINT_ROBOT_RADIUS


def almost_equal(x,y,threshold=0.1):
    return abs(x-y) < threshold

@pytest.fixture
def pr_hgraph():
    return PointRobotVelHGraph(Object(
        "emtpy obj",
        State(pos=np.array([.01, .01, .01])),
        CylinderObstacle(name="pointRobot-vel-v7-obst", content_dict={
                "type": "cylinder",
                "geometry": {"radius": 0.5, "height": 0.25},
                }),
        ), "env")

def test_find_push_pose_againts_object_state_test(pr_hgraph):
    blocking_obst = Object(
                "blocking_obj",
                State(),
                CylinderObstacle(name="blocking_obj", content_dict={
                    "type": "cylinder",
                    "geometry": {"radius": 1, "height": 0.25},
                    })
                )
    blocking_obst.type = UNMOVABLE

    pr_hgraph.setup({}, {"obst1": blocking_obst})

    path_going_north = [[0.01, 0.01], [-1, 0],[-2, 0],[-3, 0],[-4, 0]]
    path_going_south = [[0.01, 0.01], [1, 0],[2, 0],[3, 0],[4, 0]]
    path_going_west  = [[0.01, 0.01], [0, -1],[0, -2],[0, -3],[0, -4]]
    path_going_east  = [[0.01, 0.01], [0, 1],[0, 2],[0, 3],[0, 4]]

    paths = [path_going_north, path_going_south, path_going_west, path_going_east]
    best_push_pose_answers = [[1+POINT_ROBOT_RADIUS, 0], [-1-POINT_ROBOT_RADIUS, 0], [0, 1+POINT_ROBOT_RADIUS],  [0, -1-POINT_ROBOT_RADIUS]]

    for (i, path) in enumerate(paths):
        temp_push_pose =  pr_hgraph.find_push_pose_againts_object_state(blocking_obst, path).get_xy_position()

        assert almost_equal(best_push_pose_answers[i][0], temp_push_pose[0])
        assert almost_equal(best_push_pose_answers[i][1], temp_push_pose[1])
