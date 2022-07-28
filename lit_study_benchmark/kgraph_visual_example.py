from robot_brain.graph.KGraph import KGraph
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.Edge import Edge

def main():
    # create a knowledge graph and visualise

    kgraph = KGraph()

    # the robot
    node1 = ObjectSetNode(1, "robot", [])
    kgraph.addNode(node1)
    node2 = ChangeOfConfSetNode(2, "position", [])
    kgraph.addNode(node2)
    kgraph.addEdge(Edge("id", 1, 2, "MPC", "PEM"))
    # kgraph.addEdge(Edge("id", 1, 2, "MPC", "IPEM"))

    # adding expanded start and target node
    node3 = ObjectSetNode(3, "robot_and_red_sphere", [])
    node4 = ChangeOfConfSetNode(4, "box position", [])
    kgraph.addNode(node3)
    kgraph.addNode(node4)
    # kgraph.addNode(ObjectSetNode(5, "unknown_object", []))
    
    kgraph.addEdge(Edge("id", 3, 4, "EMPPI", "LSTM")) 

    kgraph.visualise()

if __name__ == "__main__":
    main()

