from robot_brain.graph.k_graph import KGraph
from robot_brain.graph.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.graph.object_set_node import ObjectSetNode
from robot_brain.graph.udge import Edge

def main():
    # create a knowledge graph and visualise

    kgraph = KGraph()

    # robot
    node1 = ObjectSetNode(1, "robot", [])
    kgraph.addNode(node1)
    node2 = ChangeOfConfSetNode(2, "robot position", [])
    kgraph.addNode(node2)
    kgraph.addEdge(Edge("id", 1, 2, "MPC", "PEM"))
 
    # blue cube
    node5 = ObjectSetNode(5, "robot_and_blue_cube", [])
    node6 = ChangeOfConfSetNode(6, "cube position", [])
    kgraph.addNode(node5)
    kgraph.addNode(node6)
    kgraph.addEdge(Edge("id", 5, 6, "RMPPI", "forward model"))

    # duck
    node3 = ObjectSetNode(7, "robot_and_duck", [])
    node4 = ChangeOfConfSetNode(8, "duck position", [])
    kgraph.addNode(node3)
    kgraph.addNode(node4)
    kgraph.addEdge(Edge("id", 7, 8, "MPC", "model fitting"))
    kgraph.visualise()

if __name__ == "__main__":
    main() 
