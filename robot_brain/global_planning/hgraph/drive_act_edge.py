import numpy as np
from robot_brain.object import Object

from robot_brain.controller.controller import Controller

from robot_brain.global_planning.hgraph.action_edge import ActionEdge
from robot_brain.global_variables import CREATE_SERVER_DASHBOARD, DT
from robot_brain.state import State

class DriveActionEdge(ActionEdge):
    """ Drive action edge controls all driving actions. """

    def __init__(self,
            iden: int,
            source: int,
            to: int,
            robot_obj: Object,
            verb: str,
            controller: Controller,
            model_name: str,
            subtask_name: str):

        ActionEdge.__init__(self, iden, source, to, robot_obj, verb, controller, model_name, subtask_name)

    def view_completed(self) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """

        if self.controller.order == 2:
            return self.robot_obj.state.position_euclidean(self.get_current_view()) < 0.4
        else:
            return self.robot_obj.state.pose_euclidean(self.get_current_view()) < 0.4

    def respond(self) -> np.ndarray:
        """ respond to the current state. """
        if self.view_completed():
            self.increment_current_view()

        return self.controller.respond(self.robot_obj.state)
