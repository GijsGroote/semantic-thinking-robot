import numpy as np
import pytest
from motion_planning_env.cylinder_obstacle import CylinderObstacle
from motion_planning_env.box_obstacle import BoxObstacle
from robot_brain.state import State
from robot_brain.object import Object
from robot_brain.global_planning.hgraph.object_node import ObjectNode
from robot_brain.global_planning.hgraph.point_robot_vel_hgraph import PointRobotVelHGraph


boxObstacle= BoxObstacle(name="None-Type-Object", content_dict={
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

def test_adding_nodes():

    node1 = ObjectNode(1, "P", Object("node1", State(), boxObstacle))
    node2 = ObjectNode(2, "P", Object("node1", State(), boxObstacle))
    node3 = ObjectNode(3, "P", Object("node1", State(), boxObstacle))

    hgraph = PointRobotVelHGraph(Object("point_robot", State(), CylinderObstacle(name="simple_cilinder", content_dict={
                    "movable": True,
                    "mass": 1000,
                    "type": "cylinder",
                    "color": [0/255, 255/255, 0/255, 1],
                    "position": [-1, -1, 0.5],
                    "geometry": {"radius": 0.5, "height": 0.25},
                    })))

    hgraph.add_node(node1)
    hgraph.add_node(node2)
    hgraph.add_node(node3)

    assert len(hgraph.nodes) == 3
