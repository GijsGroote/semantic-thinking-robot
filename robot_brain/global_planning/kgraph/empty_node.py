"""
Empty node serves the purpose of being a placeholder for an FeedbackEdge to point toward.
"""

from robot_brain.global_planning.node import Node

class EmptyNode(Node):
    """
    Empty node.
    """

    def __init__(self, iden):
        Node.__init__(self, iden)

    def to_string(self):
        return f"EmptyNode identifier: {self.iden}"
