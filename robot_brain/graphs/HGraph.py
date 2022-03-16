from robot_brain.graphs.Graph import Graph


class HGraph(Graph):

    def __init__(self):
        super().__init__()
        self.target_node = None

    def addNode(self, node):
        # todo: check this node is a valid objectSetNode
        self.nodes.append(node)

    def addTargetNode(self, node):
        # todo: check this node is a valid objectSetNode
        self.addNode(node)
        self.target_node = node


