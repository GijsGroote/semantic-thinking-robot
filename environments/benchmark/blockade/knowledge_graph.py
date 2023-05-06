from  robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.kgraph.change_of_state_node import ChangeOfStateNode
from robot_brain.global_planning.object_node import ObjectNode
from robot_brain.global_planning.edge import Edge
from robot_brain.global_planning.state import StateNode

def main():

    # create a knowledge graph and visualise
    kgraph = KGraph()

    # robot
    node1 = ObjectNode(1, "robot", [])
    kgraph.add_node(node1)
    node2 = ChangeOfStateNode(2, "robot position", [])
    kgraph.add_node(node2)
    kgraph.add_edge(Edge("id", 1, 2, "MPC", "PEM"))

    # cube 
    node3 = ObjectNode(3, "robot_and_red_cube", [])
    node4 = ChangeOfStateNode(4, "cube position", [])
    kgraph.add_node(node3)
    kgraph.add_node(node4)
    kgraph.add_edge(Edge("id", 3, 4, "MPPI", "forward model"))
   
    # wall
    node3 = ObjectNode(5, "robot_and_green_wall", [])
    node4 = ChangeOfStateNode(6, "wall position", [])
    kgraph.add_node(node3)
    kgraph.add_node(node4)
    kgraph.add_edge(Edge("id", 5, 6, "unmovable", "unmovable"))
   
    # duck
    node3 = ObjectNode(7, "robot_and_duck", [])
    node4 = ChangeOfStateNode(8, "duck position", [])
    kgraph.add_node(node3)
    kgraph.add_node(node4)
    kgraph.add_edge(Edge("id", 7, 8, "RMPPI", "LSTM"))
    

    kgraph.visualise()

if __name__ == "__main__":
    main()
