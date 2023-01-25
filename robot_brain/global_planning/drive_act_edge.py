import numpy as np

from robot_brain.global_planning.action_edge import ActionEdge
from robot_brain.state import State

class DriveActionEdge(ActionEdge):
    """ Drive action edge controls all driving actions. """

    def __init__(self, iden, source, to, verb, controller):
        ActionEdge.__init__(self, iden, source, to, verb, controller)

    def view_completed(self, state: State) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """
        print(f' under 0.5? {state.pose_euclidean(self.get_current_target())}')
        return state.pose_euclidean(self.get_current_target()) < 0.5

    def respond(self, robot_state: State) -> np.ndarray:
        """ respond to the current state. """
        return self.controller.respond(robot_state)
