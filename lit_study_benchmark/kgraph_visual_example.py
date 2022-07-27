from robot_brain.graph.KGraph import KGraph
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.Edge import Edge

def main():
    # create a knowledge graph and visualise

    kgraph = KGraph()

    node1 = ObjectSetNode(1, "start robot and box", [])
    kgraph.addNode(node1)
    node2 = ChangeOfConfSetNode(2, "target box", [])
    kgraph.addNode(node2)

        # adding expanded start and target node
    node1 = ObjectSetNode(1, "start robot", [])
    node2 = ObjectSetNode(2, "start box", [])
    node3 = ChangeOfConfSetNode(3, "target box", [])
    kgraph.addNode(node3)

    kgraph.visualiseHGraph()

if __name__ == "__main__":
    main()

