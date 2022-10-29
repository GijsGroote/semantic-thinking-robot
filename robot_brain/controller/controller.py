from abc import ABC, abstractmethod
from robot_brain.global_variables import PLOT_CONTROLLER, CREATE_SERVER_DASHBOARD, DT
from robot_brain.state import State

class Controller(ABC):
    """
    Controller class provides various single- and multi-body
    controllers, which calculate input for the robot.
    """

    def __init__(self, order):
        self.order = order
        self.dt_counter = 0
        self.target_state = State()
    
    def setup(self, dyn_model, current_state, target_state):
        self.target_state = target_state
        self._setup(dyn_model, current_state)

    @abstractmethod
    def _setup(self, model, current_state):
        pass

    def respond(self, current_state):
        """ respond with input for the robot and update dashboard every second. """

        if CREATE_SERVER_DASHBOARD and PLOT_CONTROLLER:
            if self.dt_counter % (1/DT) == 0 and self.dt_counter != 0:
                self._update_db()
            self.dt_counter += 1

        return self._find_input(current_state)

    def set_target_state(self, target_state):
        self.target_state = target_state
        self._set_target_state()

    @abstractmethod
    def _set_target_state(self):
        pass

    @abstractmethod
    def visualise(self):
        pass

    @property
    def target_state(self) -> State:
        return self._target_state

    @target_state.setter
    def target_state(self, target_state):
        assert isinstance(target_state, State), f" target_state should be a State and is {type(target_state)}" 
        self._target_state = target_state

    # TODO: order of system here?

    # private methods below this point
    @abstractmethod
    def _find_input(self, current_state):
        pass

    @abstractmethod
    def _update_db(self):
        pass
