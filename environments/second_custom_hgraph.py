import numpy as np

from robot_brain.global_planning.hgraph.point_robot_vel_hgraph import PointRobotVelHGraph

from robot_brain.global_planning.node import Node, NODE_COMPLETED, NODE_UNFEASIBLE, NODE_INITIALISED
from robot_brain.global_planning.obstacle_node import ObstacleNode
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.obstacle import Obstacle
from robot_brain.state import State

from robot_brain.global_planning.drive_ident_edge import DriveIdentificationEdge
from robot_brain.global_planning.edge import Edge, EDGE_INITIALISED, EDGE_EXECUTING, EDGE_FAILED, EDGE_COMPLETED
from robot_brain.global_planning.drive_act_edge import DriveActionEdge
from robot_brain.global_planning.push_ident_edge import PushIdentificationEdge
from robot_brain.global_planning.push_act_edge import PushActionEdge
from robot_brain.global_planning.action_edge import ActionEdge, EDGE_PATH_EXISTS, EDGE_PATH_IS_PLANNED, EDGE_HAS_SYSTEM_MODEL
from robot_brain.global_planning.hgraph.local_planning.graph_based.path_estimator import PathEstimator
from robot_brain.global_planning.identification_edge import IdentificationEdge
from robot_brain.global_planning.empty_edge import EmptyEdge
from robot_brain.controller.push.push_controller import PushController
from robot_brain.controller.drive.drive_controller import DriveController
from robot_brain.system_model import SystemModel
from robot_brain.controller.controller import Controller

def main():
    """ This HGraph is a handcoded version that represents pushing an object to a target
    location. Thus  a task with a single subtask in it. """

    robot_obst = Obstacle("robot", State(), "empty")
    hgraph = PointRobotVelHGraph(robot_obst, "env")

    # robot
    robot_node = ObstacleNode(0, "robot", robot_obst)
    hgraph.add_start_node(robot_node)


    robot_target_node = ObstacleNode(1, "robot_target", robot_obst)
    hgraph.add_target_node(robot_target_node)
#

    robot_model_node1 = ObstacleNode(2, "robot_model_1", robot_obst)
    hgraph.add_node(robot_model_node1)
    robot_model_node2 = ObstacleNode(3, "robot_model_2", robot_obst)
    hgraph.add_node(robot_model_node2)

    # green_box green_box_start_node = ObstacleNode(1, "box", Obstacle("box", State(), "empty"))
    class controller:
        def __init__(self, name):
            self.name = name

    # #
    # robot drive to box
    drive_to_box_edge = DriveActionEdge(
            hgraph.unique_edge_iden(),
            robot_model_node1.iden,
            robot_target_node.iden,
            robot_obst,
            "driving",
            controller("MPC"),
            "LTI model")

    drive_to_box_edge.status = EDGE_FAILED
    hgraph.add_edge(drive_to_box_edge)
    # hgraph.hypothesis.append(drive_to_box_edge)

    # drive ident edge
    drive_ident_edge = DriveIdentificationEdge(
            hgraph.unique_edge_iden(),
            robot_node.iden,
            robot_model_node1.iden,
            "Sys. Iden.",
            controller("MPC"),
            0,
            "str")
    drive_ident_edge.status = EDGE_FAILED
    hgraph.add_edge(drive_ident_edge)


    drive_to_box_edge2 = DriveActionEdge(
            hgraph.unique_edge_iden(),
            robot_model_node2.iden,
            robot_target_node.iden,
            robot_obst,
            "driving",
            controller("MPC"),
            "LTI model")

    drive_to_box_edge2.status = EDGE_FAILED
    drive_to_box_edge.status = EDGE_FAILED
    hgraph.add_edge(drive_to_box_edge2)
    # hgraph.hypothesis.append(drive_to_box_edge2)

    # drive ident edge
    drive_ident_edge2 = DriveIdentificationEdge(
            hgraph.unique_edge_iden(),
            robot_node.iden,
            robot_model_node2.iden,
            "Sys. Iden.",
            controller("MPC"),
            0,
            "str")
    drive_ident_edge2.status = EDGE_FAILED
    hgraph.add_edge(drive_ident_edge2)
    # hgraph.hypothesis.append(drive_ident_edge2)


    # hgraph.hypothesis.append(drive_ident_edge2)

    # hgraph.current_node = robot_model_node2

    hgraph.visualise(save=False)


if __name__ == "__main__":
        main()
