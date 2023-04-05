import numpy as np

from robot_brain.global_planning.hgraph.point_robot_vel_hgraph import PointRobotVelHGraph

from robot_brain.global_planning.node import Node, NODE_COMPLETED, NODE_UNFEASIBLE, NODE_INITIALISED
from robot_brain.global_planning.hgraph.object_node import ObjectNode
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.object import Object
from robot_brain.state import State

from robot_brain.global_planning.hgraph.drive_ident_edge import DriveIdentificationEdge
from robot_brain.global_planning.hgraph.edge import Edge, EDGE_INITIALISED, EDGE_EXECUTING, EDGE_FAILED, EDGE_COMPLETED
from robot_brain.global_planning.hgraph.drive_act_edge import DriveActionEdge
from robot_brain.global_planning.hgraph.push_ident_edge import PushIdentificationEdge
from robot_brain.global_planning.hgraph.push_act_edge import PushActionEdge
from robot_brain.global_planning.hgraph.action_edge import ActionEdge, EDGE_PATH_EXISTS, EDGE_PATH_IS_PLANNED, EDGE_HAS_SYSTEM_MODEL
from robot_brain.local_planning.graph_based.path_estimator import PathEstimator
from robot_brain.global_planning.hgraph.identification_edge import IdentificationEdge
from robot_brain.global_planning.hgraph.empty_edge import EmptyEdge
from robot_brain.controller.push.push_controller import PushController
from robot_brain.controller.drive.drive_controller import DriveController
from robot_brain.system_model import SystemModel
from robot_brain.controller.controller import Controller



def main():
    """ This HGraph is a handcoded version that represents pushing an object to a target
    location. Thus  a task with a single subtask in it. """

    robot_obst = Object("robot", State(), "empty")
    hgraph = PointRobotVelHGraph(robot_obst, "env")

    # robot
    robot_node = ObjectNode(0, "robot", robot_obst)

    hgraph.add_start_node(robot_node)

    # green_box
    green_box_start_node = ObjectNode(1, "box", Object("box", State(), "empty"))
    hgraph.add_start_node(green_box_start_node)
    green_box_target_node = ObjectNode(2, "box_target", Object("box", State(), "empty"))
    hgraph.add_target_node(green_box_target_node)

    class controller:
        def __init__(self, name):
            self.name = name

    # push action edge
    push_box_edge = PushActionEdge(
                    iden=hgraph.unique_edge_iden(),
                    source=green_box_start_node.iden,
                    to=green_box_target_node.iden,
                    robot_obst=robot_obst,
                    push_obst=green_box_start_node.object,
                    verb="pushing",
                    controller=controller("mpc"),
                    model_name="model_name")
    hgraph.add_edge(push_box_edge)

    hgraph.hypothesis.append(push_box_edge)
    # push_box_edge.status = EDGE_COMPLETED
    #
    # model node
    green_box_model_node = ObjectNode(
            hgraph.unique_node_iden(),
            "box_model",
            Object("box", State(), "empty"))

    hgraph.add_node(green_box_model_node)

    # push ident edge
    push_ident_edge = PushIdentificationEdge(
            hgraph.unique_edge_iden(),
            green_box_start_node.iden,
            green_box_model_node.iden,
            "Syst. Iden.",
            "controller",
            push_box_edge.iden,
            "sys_model_name")
    hgraph.add_edge(push_ident_edge)
    push_ident_edge.status  = EDGE_COMPLETED
    #
    # # rewire push edge
    # hgraph.hypothesis.append(push_ident_edge)
    push_box_edge.source = green_box_model_node.iden
    # #
    # robot drive to box
    drive_to_box_edge = DriveActionEdge(
            hgraph.unique_edge_iden(),
            robot_node.iden,
            green_box_start_node.iden,
            robot_obst,
            "driving",
            controller("MPC"),
            "LTI model")

    drive_to_box_edge.status = EDGE_COMPLETED
    hgraph.add_edge(drive_to_box_edge)
    # hgraph.hypothesis.append(drive_to_box_edge)

    robot_model_node = ObjectNode(
            hgraph.unique_node_iden(),
            "robot_model",
            Object("robot_model", State(), "empty"))

    hgraph.add_node(robot_model_node)
    #
    # drive ident edge
    drive_ident_edge = DriveIdentificationEdge(
            hgraph.unique_edge_iden(),
            robot_node.iden,
            robot_model_node.iden,
            "Sys. Iden.",
            controller("MPC"),
            0,
            "str")
    drive_ident_edge.status = EDGE_COMPLETED
    hgraph.add_edge(drive_ident_edge)
    # hgraph.hypothesis.append(drive_ident_edge)

    # # rewire drive edge
    drive_to_box_edge.source = robot_model_node.iden
    # #
    # best pose node
    # best_push_node = ObjectNode(
    #         hgraph.unique_node_iden(),
    #         "best_push_position",
    #         Object("-", State(), "empty"))
    #
    # hgraph.add_node(best_push_node)


    # robot_copy_node = ObjectNode(9, "robot_copy", robot_obst)
    # hgraph.add_node(robot_copy_node)

    # robot drive to best push pose
    # drive_to_best_push_edge = DriveActionEdge(
    #         hgraph.unique_edge_iden(),
    #         robot_copy_node.iden,
    #         best_push_node.iden,
    #         robot_obst,
    #         "driving",
    #         controller("MPC"),
    #         "LTI model")

    # hgraph.hypothesis.append(drive_to_best_push_edge)
    # drive_to_best_push_edge.status = EDGE_COMPLETED

    # hgraph.add_edge(EmptyEdge(15, green_box_model_node.iden, robot_copy_node.iden))

    # hgraph.add_edge(drive_to_best_push_edge)

    # push_box_edge.source = best_push_node.iden
    #
    robot_model2_node = ObjectNode(
            hgraph.unique_node_iden(),
            "robot_model2",
            Object("robot_copy_model", State(), "empty"))

    # hgraph.add_node(robot_model2_node)
    #
    # drive ident edge
    # drive_best_push_ident_edge = DriveIdentificationEdge(
    #         hgraph.unique_edge_iden(),
    #         robot_copy_node.iden,
    #         robot_model2_node.iden,
    #         "Sys. Iden.",
    #         controller("MPC"),
    #         0,
    #         "str")
    # hgraph.add_edge(drive_best_push_ident_edge)
    # hgraph.hypothesis.append(drive_best_push_ident_edge)
    # drive_best_push_ident_edge.status = EDGE_COMPLETED
    # #
    # #
    # drive_to_best_push_edge.source = robot_model2_node.iden

    hgraph.current_node = green_box_model_node

    hgraph.visualise(save=False)


if __name__ == "__main__":
        main()
