from robot_brain.state_node import StateNode
from robot_brain.global_planning.object_node import ObjectNode
from robot_brain.global_planning.kgraph.change_of_state_node import ChangeOfStateNode
from robot_brain.global_planning.kgraph import KGraph
from robot_brain.global_planning.edge import Edge

def main():
    # create a knowledge graph and visualise

    kgraph = KGraph()

    # robot
    node1 = ObjectNode(1, "robot", [])
    kgraph.addNode(node1)
    node2 = ChangeOfStateNode(2, "robot position", [])
    kgraph.addNode(node2)
    kgraph.addEdge(Edge("id", 1, 2, "MPC", "IPEM"))

    # red cube
    node3 = ObjectNode(3, "robot_and_red_cube", [])
    node4 = ChangeOfStateNode(4, "cube position", [])
    kgraph.addNode(node3)
    kgraph.addNode(node4)
    kgraph.addEdge(Edge("id", 3, 4, "unmovable", "unmovable"))

    # blue cube
    node5 = ObjectNode(5, "robot_and_blue_cube", [])
    node6 = ChangeOfStateNode(6, "cube position", [])
    kgraph.addNode(node5)
    kgraph.addNode(node6)
    kgraph.addEdge(Edge("id", 5, 6, "unmovable", "unmovable"))

    # green cube

    node7 = ObjectNode(7, "robot_and_green_cube", [])
    node8 = ChangeOfStateNode(8, "cube position", [])
    kgraph.addNode(node7)
    kgraph.addNode(node8)
    kgraph.addEdge(Edge("id", 7, 8, "RMPPI", "LSTM"))


    kgraph.visualise()

if __name__ == "__main__":
    main()
