import numpy as np

from robot_brain.global_planning.action_edge import ActionEdge
from robot_brain.state import State

class PushActionEdge(ActionEdge):
    """ Push action edge controls all pushing actions. """

    def __init__(self, iden, source, to, robot_obst, push_obst, verb, controller, model_name):

        ActionEdge.__init__(self, iden, source, to, robot_obst, verb, controller, model_name)
        self.push_obst = push_obst

    def view_completed(self) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """

        return self.push_obst.state.pose_euclidean(self.get_current_view()) < 1.0

    def respond(self) -> np.ndarray:
        """ respond to the robot and obstacle state. """
        if self.view_completed():
            self.increment_current_view()

        return self.controller.respond(self.robot_obst.state, self.push_obst.state)
