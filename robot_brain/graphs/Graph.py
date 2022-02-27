from abc import abstractmethod


class Graph:

    def __init__(self):
        self.nodes = None
        self.edges = None

    @abstractmethod
    def getNodes(self):
        pass

#     todo: all the things a graph can do
