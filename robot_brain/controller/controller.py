from abc import ABC, abstractmethod

from robot_brain.state import State
from robot_brain.system_model import SystemModel


class Controller(ABC):
    """
    Controller class provides various single- and multi-body
    controllers, which calculate input for the robot.
    """

    def __init__(self, order: int):
        self.order = order
        self.dt_counter = 0

        self._system_model = SystemModel(None)
        self.target_state = State()
        self.pred_error = []

    def set_target_state(self, target_state: State):
        """ update controller to steer toward a new target state. """
        self.target_state = target_state
        self._set_target_state()

    @abstractmethod
    def _set_target_state(self):
        """ sets the target state. """

    def update_db(self):
        self.visualise(save=True)

    @abstractmethod
    def visualise(self, save: bool):
        """ visualise the controller behaviour. """

    ##### Setters and Getters below this point #######
    @property
    def order(self) -> int:
        return self._order

    @order.setter
    def order(self, val: int):
        assert isinstance(val, int), f"order must be an integer and is {type(val)}"
        assert val > 0, f"order must be a positive integer and is {val}"
        self._order = val

    @property
    def dt_counter(self) -> int:
        return self._dt_counter

    @dt_counter.setter
    def dt_counter(self, val: int):
        assert isinstance(val, int), f"dt_counter must be an integer and is {type(val)}"
        assert val >= 0, f"dt_counter must be 0 or a positive integer and is {val}"
        self._dt_counter = val

    @property
    def system_model(self):
        return self._system_model

    @system_model.setter
    def system_model(self, val):

        assert val.name is not None,\
                "system models name should not me None"
        assert isinstance(val, SystemModel),\
                f"system model should be of type SystemModel and is {type(val)}"
        self._system_model = val

    @property
    def target_state(self) -> State:
        return self._target_state

    @target_state.setter
    def target_state(self, target_state: State):
        assert isinstance(target_state, State), f" target_state should be a State and is {type(target_state)}"
        self._target_state = target_state
