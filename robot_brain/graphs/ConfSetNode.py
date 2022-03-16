from robot_brain.graphs.Node import Node


class ConfSetNode(Node):

    def __init__(self, conf_set, iden):
        super().__init__(iden)
        self.conf_set = conf_set


# todo: the rest of this class
