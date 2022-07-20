from robot_brain.graph.Graph import Graph
from robot_brain.graph.HGraph import HGraph
from robot_brain.graph.Node import Node
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.graph.Edge import Edge

def main():
    # create a hypothesis graph and visualise
    hgraph = HGraph()


    node1 = ObjectSetNode(10, "starting position robot", [])
    node1.make_starting_node()
    node1.make_current_node()
    hgraph.addNode(node1)

    # adding nodes
    node2 = ConfSetNode(2, "starting position box", [])
    node2.make_starting_node()
    hgraph.addNode(node2)

    node3 = ConfSetNode(3, "target position box", [])
    node3.make_target_node()
    hgraph.addTargetNode(node3)


    hgraph.addNode(ConfSetNode(4, "robot next to box", []))
    hgraph.addEdge(Edge("id", 4, 2, "warmup stage", "controller"))
    hgraph.addEdge(Edge("id", 2, 3, "EMPPI", "controller"))




    hgraph.addEdge(Edge("id", 10, 4, "mpc1", "controller"))
    # hgraph.addEdge(Edge("id", 1, 4, "mpc2", "controller"))

    node5 = ConfSetNode(5, "robot with model", [])
    # node5.make_current_node()
    hgraph.addNode(node5)
    
    edge2 = Edge("id", 5, 4, "mpc2", "controller")
    # edge2.path = True
    hgraph.addEdge(edge2)
    edge1 = Edge("id", 10, 5, "PEM", "controller")
    # edge1.path = True
    hgraph.addEdge(edge1)


    hgraph.visualiseHGraph()

if __name__ == "__main__":
    main()
