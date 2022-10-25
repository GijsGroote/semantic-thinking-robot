from robot_brain.global_planning.obstacle_node import ObstacleNode
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.global_planning.kgraph import KGraph
from robot_brain.global_planning.edge import Edge

def main():
    # create a knowledge graph and visualise

    kgraph = KGraph()

    # robot
    node1 = ObstacleNode(1, "robot", [])
    kgraph.add_node(node1)
    node2 = ChangeOfStateNode(2, "robot position", [])
    kgraph.add_node(node2)
    kgraph.add_edge(Edge("id", 1, 2, "MPC", "PEM"))
    # kgraph.add_edge(Edge("id", 1, 2, "MPC", "PEM"))
 
    # blue cube
    # node5 = ObstacleNode(5, "robot_and_blue_cube", [])
    node5 = ObstacleNode(5, "robot_and_red_sphere", [])
    node6 = ChangeOfStateNode(6, "cube position", [])
    kgraph.add_node(node5)
    kgraph.add_node(node6)
    kgraph.add_edge(Edge("id", 5, 6, "RMPPI", "forward model"))

    # duck
    node3 = ObstacleNode(7, "robot_and_duck", [])
    node4 = ChangeOfStateNode(8, "duck position", [])
    kgraph.add_node(node3)
    kgraph.add_node(node4)
    kgraph.add_edge(Edge("id", 7, 8, "MPC", "model fitting"))
    kgraph.visualise()

if __name__ == "__main__":
    main() 
