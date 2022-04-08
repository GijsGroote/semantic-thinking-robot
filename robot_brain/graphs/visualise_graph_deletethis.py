from Graph import Graph
from HGraph import HGraph
from Node import Node
from ConfSetNode import ConfSetNode
from ObjectSetNode import ObjectSetNode
from ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.graphs.Edge import Edge


def main():

    hgraph = HGraph()

    # target should be a confsetnode
    hgraph.addNode(ConfSetNode(2, "P", []))
    hgraph.addTargetNode(ConfSetNode(2, "P", []))

    # adding a node can be confset, objectnodeset or changeInConfsetNode
    confnode = ChangeOfConfSetNode(3, "P", [])


    hgraph.addNode(confnode)
    hgraph.addNode(ChangeOfConfSetNode(7, "P", []))

    hgraph.addNode(ConfSetNode(1, "P", []))
    hgraph.addNode(ObjectSetNode(5, "P", []))

    hgraph.addEdge(Edge("id", 2, 3, "mpc", "controller"))
    hgraph.addEdge(Edge("id", 7, 1, "pid", "controller"))
    hgraph.addEdge(Edge("id", 3, 3, "EMPPI", "controller"))
    hgraph.addEdge(Edge("id", 7, 5, "mpc", "controller"))


    hgraph.visualise()

if __name__ == "__main__":
    main()
