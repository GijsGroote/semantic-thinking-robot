from robot_brain.graphs.Node import Node


class ObjectSetNode(Node):

    def __init__(self, object_set, iden):
        super().__init__(iden)
        self.object_set = object_set


    def addNode(self):
        self.ob

