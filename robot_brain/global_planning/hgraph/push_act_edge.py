import numpy as np

from robot_brain.object import Object

from robot_brain.controller.controller import Controller
from robot_brain.global_planning.hgraph.action_edge import ActionEdge
from robot_brain.exceptions import PushAnUnmovableObjectException

class PushActionEdge(ActionEdge):
    """ Push action edge controls all pushing actions. """

    def __init__(self, iden: int,
            source: int,
            to: int,
            robot_obj: Object,
            push_obj: Object,
            verb: str,
            controller: Controller,
            subtask_name: str,
            model_name: str,
            check_obj_movable: bool):

        self.check_obj_movable = check_obj_movable

        if check_obj_movable:
            self.start_counter = 0
            self.obj_start_2d_pose = push_obj.state.get_2d_pose()

        ActionEdge.__init__(self, iden, source, to, robot_obj, verb, controller, subtask_name, model_name)

        self.push_obj = push_obj

    def view_completed(self) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """

        return self.push_obj.state.position_euclidean(self.get_current_view()) < 0.9

    def respond(self) -> np.ndarray:
        """ respond to the robot and obstacle state. """
        if self.check_obj_movable:
            # I do assume that in 30 time steps the object should be moved if it is movable
            if self.start_counter == 50:
                if all(self.obj_start_2d_pose == self.push_obj.state.get_2d_pose()):
                    raise PushAnUnmovableObjectException()

        if self.check_obj_movable:
            self.start_counter += 1

        if self.view_completed():
            self.increment_current_view()

        return self.controller.respond(self.robot_obj.state, self.push_obj.state)
