import numpy as np
from robot_brain.controller.controller import Controller
from robot_brain.global_planning.action_edge import ActionEdge

class DriveActionEdge(ActionEdge):
    """ Drive action edge controls all driving actions. """

    def __init__(self, iden, source, to, verb, controller):
        ActionEdge.__init__(self, iden, source, to, verb, controller)


