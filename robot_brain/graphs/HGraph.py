from robot_brain.graphs.Graph import Graph


class HGraph(Graph):

    def __init__(self):
        Graph.__init__(self)
        self.nodes = None
        self.edges = None

    def getNodes(self):
        print("must return all the nodes of the hypothesis graph")


