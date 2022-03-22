from Graph import Graph
from HGraph import HGraph
from ConfSetNode import ConfSetNode
from ObjectSetNode import ObjectSetNode
from robot_brain.graphs.Edge import Edge


def main():

    hgraph = HGraph()
    print(type(hgraph))
    hgraph.addTargetNode(ConfSetNode(2, "P", []))
    hgraph.addNode(ConfSetNode(3, "P", []))
    hgraph.addNode(ConfSetNode(7, "P", []))
    hgraph.addNode(ConfSetNode(1, "P", []))
    hgraph.addNode(ObjectSetNode(5, "P", []))
    hgraph.addEdge(Edge("id",2,3,"verb", "controller"))
    hgraph.addEdge(Edge("id", 7, 1, "verb", "controller"))
    hgraph.addEdge(Edge("id", 3, 3, "verb", "controller"))
    hgraph.addEdge(Edge("id", 7, 5, "verb", "controller"))


    hgraph.visualise()

if __name__ == "__main__":
    main()
