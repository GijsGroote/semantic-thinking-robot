"""
Empty node serves the purpose of being a placeholder for an FeedbackEdge to point toward.
"""

from robot_brain.global_planning.node import Node

class EmptyNode(Node):
    """
    Empty node.
    """

    def __init__(self, iden: int, name: str):
        Node.__init__(self, iden)
        self.name = name + "_emtpy_node"

    def to_string(self):
        return f"EmptyNode identifier: {self.iden}, and name: {self.name}"
