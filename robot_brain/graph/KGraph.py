from robot_brain.graph.Graph import Graph


class KGraph(Graph):

    def __init__(self):
        Graph.__init__(self)
        self.is_class = "kgraph"
        # print("ha a knowledge graphs yo")


    def addNode(self, node):
        self.nodes.append(node)
