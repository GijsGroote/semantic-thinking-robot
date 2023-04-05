from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.global_planning.hgraph.object_node import ObjectNode
from robot_brain.object import Object
from robot_brain.state import State

from robot_brain.global_planning.drive_act_edge import DriveActionEdge

class controller:
    def __init__(self, name):
        self.name = name

def main():
    # create a knowledge graph and visualise

    kgraph = KGraph()
    obst = Object('Object1', State(), "empty")

    # 5 nodes, robot object and 3 neutral
    node1 = ObjectNode(1, "point_robot", obst)
    node2 = ObjectNode(2, "green_box", obst)
    node3 = ChangeOfStateNode(3, "robot_and_green_wall", [])
    node4 = ChangeOfStateNode(4, "robot_and_green_wall", [])
    node5 = ChangeOfStateNode(5, "robot_and_green_wall", [])
    node6 = ChangeOfStateNode(6, "robot_and_green_wall", [])
    node7 = ChangeOfStateNode(7, "robot_and_green_wall", [])
    kgraph.add_node(node1)
    kgraph.add_node(node2)
    kgraph.add_node(node3)
    kgraph.add_node(node4)
    kgraph.add_node(node5)
    kgraph.add_node(node6)
    kgraph.add_node(node7)

    kgraph.add_edge(DriveActionEdge(iden=1,
        source=1,
        to=3,
        robot_obst="robot",
        verb="MPC, LTI model1",
        controller=controller("mpc"),
        model_name="LTI ss"))

    kgraph.add_edge(DriveActionEdge(iden=2,
        source=1,
        to=4,
        robot_obst="robot",
        verb="MPC, LTI model2",
        controller=controller("mpc"),
        model_name="LTI ss"))

    kgraph.add_edge(DriveActionEdge(iden=3,
        source=1,
        to=5,
        robot_obst="robot",
        verb="MPPI, nonlinear model1",
        controller=controller("mppi"),
        model_name="LTI ss2"))

    kgraph.add_edge(DriveActionEdge(iden=4,
        source=2,
        to=7,
        robot_obst="robot",
        verb="MPC, LTI model3",
        controller=controller("mpc"),
        model_name="LTI ss3"))

    kgraph.add_edge(DriveActionEdge(iden=4,
        source=2,
        to=6,
        robot_obst="robot",
        verb="MPPI, nonlinear model2",
        controller=controller("mpc"),
        model_name="LTI ss3"))


    kgraph.visualise()

if __name__ == "__main__":
    main()
