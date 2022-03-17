from robot_brain.graphs.Node import Node


class ObjectSetNode(Node):

    def __init__(self, iden, state):
        super().__init__(iden)
        self.state = state
        # todo: add object set
        # self.object_set = object_set



