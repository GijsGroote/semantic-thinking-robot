from abc import abstractmethod
from pyvis.network import Network


class Graph:

    def __init__(self):
        self.nodes = []
        self.edges = []

    def getNodes(self):
        return self.nodes

    @abstractmethod
    def addNode(self):
        pass

    def getEdges(self):
        return self.edges

    def addEdge(self, edge):
        self.edges.append(edge)


    def visualise(self):
        # make this function such that it updates if it is already present
        net = Network()

        # add nodes
        for node in self.getNodes():
            print(node.iden)
            net.add_node(node.iden, label=node.iden)

        # add edges
        for edge in self.getEdges():
            net.add_edge(edge.source, edge.to)

        net.show("../../dashboard/graph.html")





#     todo: all the things a graph can do
