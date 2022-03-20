from Graph import Graph
from HGraph import HGraph
from ConfSetNode import ConfSetNode
from ObjectSetNode import ObjectSetNode


def main():
    print("create Hgraph")

    hgraph = HGraph()
    hgraph.addTargetNode(ConfSetNode(2, "P", []))
    hgraph.addNode(ObjectSetNode(1, "P", []))

    hgraph.visualise()

if __name__ == "__main__":
    main()
