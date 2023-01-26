import numpy as np

from robot_brain.global_planning.action_edge import ActionEdge
from robot_brain.state import State

class PushActionEdge(ActionEdge):
    """ Push action edge controls all pushing actions. """

    def __init__(self, iden, source, to, verb, controller):
        ActionEdge.__init__(self, iden, source, to, verb, controller)

    def view_completed(self, state: State) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """

        print(f'wat is de error? = {state.pose_euclidean(self.get_current_target())}')
        return state.pose_euclidean(self.get_current_target()) < 0.8

    def respond(self, robot_state: State, obst_state=None) -> np.ndarray:
        """ respond to the robot and obstacle state. """
        return self.controller.respond(robot_state, obst_state=obst_state)
