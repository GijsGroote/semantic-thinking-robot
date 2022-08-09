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
    kgraph.addEdge(Edge("id", 1, 2, "MPC", "PEM"))

    # cube 
    node3 = ObjectSetNode(3, "robot_and_red_cube", [])
    node4 = ChangeOfConfSetNode(4, "cube position", [])
    kgraph.addNode(node3)
    kgraph.addNode(node4)
    kgraph.addEdge(Edge("id", 3, 4, "MPPI", "forward model"))
   
    # wall
    node3 = ObjectSetNode(5, "robot_and_green_wall", [])
    node4 = ChangeOfConfSetNode(6, "wall position", [])
    kgraph.addNode(node3)
    kgraph.addNode(node4)
    kgraph.addEdge(Edge("id", 5, 6, "unmovable", "unmovable"))
   
    # duck
    node3 = ObjectSetNode(7, "robot_and_duck", [])
    node4 = ChangeOfConfSetNode(8, "duck position", [])
    kgraph.addNode(node3)
    kgraph.addNode(node4)
    kgraph.addEdge(Edge("id", 7, 8, "RMPPI", "LSTM"))
    

    kgraph.visualise()

if __name__ == "__main__":
    main()
