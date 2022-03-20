from robot_brain.graphs.Graph import Graph


class KGraph(Graph):

    def __init__(self):
        super().__init__()
        print("ha a knowledge graphs yo")


    def addNode(self, node):
        # todo: input sanitization
        self.nodes.append(node)
