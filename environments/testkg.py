from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.global_planning.obstacle_node import ObstacleNode
from robot_brain.obstacle import Obstacle
from robot_brain.state import State

from robot_brain.global_planning.drive_act_edge import DriveActionEdge

class controller:
    def __init__(self, name):
        self.name = name


def main():
    # create a knowledge graph and visualise

    kgraph = KGraph()
    obst = Obstacle('Obstacle1', State(), "empty")

    # 5 nodes, robot obstacle and 3 neutral
    node1 = ObstacleNode(1, "point_robot", obst)
    node2 = ObstacleNode(2, "green_box", obst)
    node3 = ChangeOfStateNode(3, "robot_and_green_wall", [])
    node4 = ChangeOfStateNode(4, "robot_and_green_wall", [])
    node5 = ChangeOfStateNode(5, "robot_and_green_wall", [])
    kgraph.add_node(node1)
    kgraph.add_node(node2)
    kgraph.add_node(node3)
    kgraph.add_node(node4)
    kgraph.add_node(node5)

    kgraph.add_edge(DriveActionEdge(1, 1, 3, "MPC, LTI model2",  controller("mpc")))
    kgraph.add_edge(DriveActionEdge(2, 1, 4, "MPC, LTI model1",  controller("mpc")))
    kgraph.add_edge(DriveActionEdge(3, 2, 5, "MPPI, nonlinear model",  controller("mpc")))

    kgraph.visualise()

if __name__ == "__main__":
    main()
