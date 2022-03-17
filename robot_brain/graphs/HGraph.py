# from robot_brain.graphs.Graph import Graph
from pyvis.network import Network
import numpy as np


class HGraph:

    def __init__(self):
        self.nodes = []
        self.edges = []
        self.target_node = None

    def addNode(self, node):
        # todo: check this node is a valid objectSetNode
        self.nodes = node

    def addEdge(self, edge):
        self.edges = edge

    def addTargetNode(self, node):
        # todo: check this node is a valid objectSetNode

        self.addNode(self, node)
        self.target_node = node

    def getNodes(self):
        return self.nodes

    def visualise(self):
        # make this function such that it updates if it is already present
        net = Network()

        # add nodes
        for node in self.getNodes(self):
            print(node.iden)
            net.add_node(node.iden, label=str(np.random(0, 10))+node.iden)

        # add edges
        for edge in self.getEdges(self):
            net.add_edge(edge.source, edge.to)

        net.show("../../dashboard/graph.html")


