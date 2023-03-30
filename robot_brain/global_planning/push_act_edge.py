import numpy as np

from robot_brain.global_planning.action_edge import ActionEdge
from robot_brain.state import State

class PushActionEdge(ActionEdge):
    """ Push action edge controls all pushing actions. """

    def __init__(self, iden, source, to, robot_obst, push_obst, verb, controller, model_name, check_obj_movable):

        self.check_obj_movable = check_obj_movable

        if check_obj_movable:
            self.start_counter = 0
            self.obj_start_2d_pose = push_obst.state.get_2d_pose()

        ActionEdge.__init__(self, iden, source, to, robot_obst, verb, controller, model_name)
        self.push_obst = push_obst

    def view_completed(self) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """

        return self.push_obst.state.pose_euclidean(self.get_current_view()) < 0.4

    def respond(self) -> np.ndarray:
        """ respond to the robot and obstacle state. """
        if self.check_obj_movable:
            print(f'yes I am chenging if it is movable {self.check_obj_movable}')
        # I do assume that in 30 time steps the object should be moved if it is movable
        if self.start_counter == 30:
            print(f'checking has it moved?{self.obj_start_2d_pose} and {self.push_obst.state.get_2d_pose()}')
            if all(self.obj_start_2d_pose == self.push_obst.state.get_2d_pose()):
                # TODO: from here, update the object to unmovable
                print('well what is this, the object is godd damn not movable3')
                print('well what is this, the object is godd damn not movable3')
                print('well what is this, the object is godd damn not movable3')
                print('well what is this, the object is godd damn not movable3')

        if self.check_obj_movable:
            self.start_counter += 1

        if self.view_completed():
            self.increment_current_view()

        return self.controller.respond(self.robot_obst.state, self.push_obst.state)
