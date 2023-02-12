import numpy as np

from robot_brain.global_planning.action_edge import ActionEdge
from robot_brain.state import State

class DriveActionEdge(ActionEdge):
    """ Drive action edge controls all driving actions. """

    def __init__(self, iden, source, to, robot_obst, verb, controller, model_name):
        ActionEdge.__init__(self, iden, source, to, robot_obst, verb, controller, model_name)


    def view_completed(self) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """

        return self.robot_obst.state.pose_euclidean(self.get_current_view()) < 0.4

    def respond(self) -> np.ndarray:
        """ respond to the current state. """
        if self.view_completed():
            self.increment_current_view()

        return self.controller.respond(self.robot_obst.state)
