import copy
import numpy as np
from robot_brain.object import Object

from robot_brain.controller.controller import Controller
from robot_brain.global_planning.hgraph.action_edge import ActionEdge
from robot_brain.exceptions import FaultDetectedException, PushAnUnmovableObjectException, PushAnMovableObjectException
from helper_functions.extract_object_info import get_max_dimension_from_object

from robot_brain.global_variables import POINT_ROBOT_RADIUS

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
        self.push_obj_start_state = copy.deepcopy(push_obj.state)

        ActionEdge.__init__(self, iden, source, to, robot_obj, verb, controller, subtask_name, model_name)
        self.push_obj = push_obj

    def view_completed(self) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """

        return self.push_obj.state.position_euclidean(self.get_current_view()) < 1.0 and\
                self.push_obj_start_state.position_euclidean(self.push_obj.state) != 0

    def respond(self) -> np.ndarray:
        """ respond to the robot and obstacle state. """
        # detect if an object is movable or unmovable
        if self.check_obj_movable:
            if not all(self.push_obj_start_state.get_2d_pose() == self.push_obj.state.get_2d_pose()):
                raise PushAnMovableObjectException

            if self.start_counter == 50 and all(self.push_obj_start_state.get_2d_pose() == self.push_obj.state.get_2d_pose()):
                raise PushAnUnmovableObjectException

            self.start_counter += 1

        # detect fault: push object deviates to far from path
        if self.push_obj.state.position_euclidean(self.get_current_view()) > 2.0:
            raise FaultDetectedException("pushing object {self.push_obj.name} deviated to far from path")

        # detect fault: robot deviates to far from push object
        if self.push_obj.state.position_euclidean(self.robot_obj.state) >\
                2*(POINT_ROBOT_RADIUS+get_max_dimension_from_object(self.push_obj)):
            raise FaultDetectedException(f"{self.robot_obj.name} deviated to far from pushable object {self.push_obj.name}")

        if self.view_completed():
            self.increment_current_view()

        return self.controller.respond(self.robot_obj.state, self.push_obj.state)
