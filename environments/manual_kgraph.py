
import math
import time
import numpy as np
from robot_brain.state import State
from robot_brain.global_planning.kgraph.kgraph import KGraph

from environments.objects.boxes import box, box2
from environments.objects.cylinders import cylinder

from robot_brain.object import Object, MOVABLE, UNMOVABLE, UNKNOWN
from robot_brain.global_planning.object_node import ObjectNode
from robot_brain.state import State
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.global_planning.edge import Edge, EDGE_INITIALISED, EDGE_COMPLETED, EDGE_EXECUTING, EDGE_FAILED
from robot_brain.global_planning.kgraph.feedback_edge import FeedbackEdge
from robot_brain.global_planning.kgraph.empty_node import EmptyNode

def main():

    kgraph = KGraph()

    robot_obj = Object(
            name='robot',
            state=State(),
            properties=CylinderObstacle(
                name="pointRobot-vel-v7-obst",
                content_dict={
                    "type": "box",
                    "geometry": {"radius": 0.22, "height": 0.25}}),
            obj_type=MOVABLE
            )

    kgraph.add_node(ObjectNode(
            iden=kgraph.unique_node_iden(),
            name="robot",
            obj=robot_obj,
            subtask_name='name'))

    kgraph.add_node(ObjectNode(
            iden=kgraph.unique_node_iden(),
            name="box",
            obj=robot_obj,
            subtask_name='name'))

    kgraph.add_node(EmptyNode(kgraph.unique_node_iden(), "side node"))
    kgraph.add_node(EmptyNode(kgraph.unique_node_iden(), "side node"))
    kgraph.add_node(EmptyNode(kgraph.unique_node_iden(), "side node"))

    contr = {"name": "temp"}
    class Contr:
        def __init__(self):
            self.name = 'saus'

    kgraph.add_edge(FeedbackEdge(
                iden=kgraph.unique_edge_iden(),
                source=0,
                to=2,
                success_factor=1.0,
                obj=robot_obj,
                verb='MPC, lti-drive-model',
                controller=Contr(),
                model_name='one',
                edge_status=EDGE_COMPLETED))
    kgraph.add_edge(FeedbackEdge(
                iden=kgraph.unique_edge_iden(),
                source=0,
                to=3,
                success_factor=1.0,
                obj=robot_obj,
                verb='MPPI, lti-drive-model',
                controller=Contr(),
                model_name='one',
                edge_status=EDGE_COMPLETED))

    kgraph.add_edge(FeedbackEdge(
                iden=kgraph.unique_edge_iden(),
                source=1,
                to=4,
                success_factor=1.0,
                obj=robot_obj,
                verb='MPPI, nonlinear-push-model-1',
                controller=Contr(),
                model_name='one',
                edge_status=EDGE_COMPLETED))

    kgraph.visualise(save=False)




if __name__ == "__main__":
    main()

