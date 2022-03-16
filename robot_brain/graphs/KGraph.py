from robot_brain.graphs.Graph import Graph


class KGraph(Graph):

    def __init__(self):
        super().__init__()
        print("ha a knowledge graphs yo")


    def addNode(self):
        print("must return Nodes, from which configuration set would you like to have those?")
