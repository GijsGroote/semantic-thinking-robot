from robot_brain.graphs.Node import Node


class ChangeOfConfSetNode(Node):

    def __init__(self, change_of_conf_set, iden):
        super().__init__(iden)
        self.change_of_conf_set = change_of_conf_set


# todo: the rest of this class
