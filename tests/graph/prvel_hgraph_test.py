
import pytest
from robot_brain.global_planning.obstacle_node import ObstacleNode
from robot_brain.obstacle import Obstacle
from robot_brain.state import State
from motion_planning_env.cylinder_obstacle import CylinderObstacle
from robot_brain.global_planning.hgraph.point_robot_vel_hgraph import PointRobotVelHGraph


@pytest.fixture
def pr_hgraph():
    return PointRobotVelHGraph(Obstacle(
        "emtpy obj",
        State(),
        CylinderObstacle(name="pointRobot-vel-v7-obst", content_dict={
                "type": "cylinder",
                "geometry": {"radius": 0.22, "height": 0.25},
                }),
        ), "env")

# TODO: make 4 push positions, and then a test that comes iwth an extra hard find push pose 
def test_find_push_pose_againts_obstacle_state_test(pr_hgraph):
    print('wi8ll you')
    blocking_obst = Obstacle(
                "emtpy obj",
                State(),
                CylinderObstacle(name="pointRobot-vel-v7-obst", content_dict={
                    "type": "cylinder",
                    "geometry": {"radius": 0.22, "height": 0.25},
                    })
                )

    print(f'this nigga has porperties {blocking_obst.properties}')

    path_going_north = [[0, 0], [-1, 0],[-2, 0],[-3, 0],[-4, 0]]
    path_going_west = [[0, 0], [0, 1],[0, 2],[0, 3],[0, 4]]
    push_pose = pr_hgraph.find_push_pose_againts_obstacle_state(blocking_obst, path_going_north)
    print(push_pose.to_string())
    assert False
    assert True

