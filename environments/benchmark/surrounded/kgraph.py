from robot_brain.graph.KGraph import KGraph
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.Edge import Edge

def main():
    # create a knowledge graph and visualise

    kgraph = KGraph()

    # robot
    node1 = ObjectSetNode(1, "robot", [])
    kgraph.addNode(node1)
    node2 = ChangeOfConfSetNode(2, "robot position", [])
    kgraph.addNode(node2)
    kgraph.addEdge(Edge("id", 1, 2, "MPC", "IPEM"))

    # red cube 
    node3 = ObjectSetNode(3, "robot_and_red_cube", [])
    node4 = ChangeOfConfSetNode(4, "cube position", [])
    kgraph.addNode(node3)
    kgraph.addNode(node4)
    kgraph.addEdge(Edge("id", 3, 4, "unmovable", "unmovable"))
   
    # blue cube
    node5 = ObjectSetNode(5, "robot_and_blue_cube", [])
    node6 = ChangeOfConfSetNode(6, "cube position", [])
    kgraph.addNode(node5)
    kgraph.addNode(node6)
    kgraph.addEdge(Edge("id", 5, 6, "unmovable", "unmovable"))
   
    # green cube

    node7 = ObjectSetNode(7, "robot_and_green_cube", [])
    node8 = ChangeOfConfSetNode(8, "cube position", [])
    kgraph.addNode(node7)
    kgraph.addNode(node8)
    kgraph.addEdge(Edge("id", 7, 8, "RMPPI", "LSTM"))  


    kgraph.visualise()

if __name__ == "__main__":
    main()
