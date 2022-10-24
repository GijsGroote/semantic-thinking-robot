from robot_brain.global_planning.kgraph import KGraph
from robot_brain.global_planning.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.global_planning.obstacle_set_node import ObstacleSetNode
from robot_brain.global_planning.edge import Edge

def main():
    # create a knowledge graph and visualise

    kgraph = KGraph()

    # robot
    node1 = ObstacleSetNode(1, "robot", [])
    kgraph.add_node(node1)
    node2 = ChangeOfConfSetNode(2, "robot position", [])
    kgraph.add_node(node2)
    kgraph.add_edge(Edge("id", 1, 2, "MPC", "PEM"))
    # kgraph.add_edge(Edge("id", 1, 2, "MPC", "PEM"))
 
    # blue cube
    # node5 = ObstacleSetNode(5, "robot_and_blue_cube", [])
    node5 = ObstacleSetNode(5, "robot_and_red_sphere", [])
    node6 = ChangeOfConfSetNode(6, "cube position", [])
    kgraph.add_node(node5)
    kgraph.add_node(node6)
    kgraph.add_edge(Edge("id", 5, 6, "RMPPI", "forward model"))

    # duck
    node3 = ObstacleSetNode(7, "robot_and_duck", [])
    node4 = ChangeOfConfSetNode(8, "duck position", [])
    kgraph.add_node(node3)
    kgraph.add_node(node4)
    kgraph.add_edge(Edge("id", 7, 8, "MPC", "model fitting"))
    kgraph.visualise()

if __name__ == "__main__":
    main() 
