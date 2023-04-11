from abc import abstractmethod
import numpy as np
from robot_brain.global_variables import (
        PLOT_CONTROLLER,
        CREATE_SERVER_DASHBOARD,
        DT,
        LOG_METRICS,
        )
from robot_brain.state import State
from robot_brain.system_model import SystemModel
from robot_brain.controller.controller import Controller

class DriveController(Controller):
    """
    Controller class for driving.
    """

    def __init__(self, order: int):
        Controller.__init__(self, order)

    def setup(self, system_model, current_state: State, target_state: State):
        """ setup the controller, this is seperated from __init__
        because dynamic models could not yet exist. """
        self.system_model = system_model
        self.target_state = target_state
        print(f"set this as target state {target_state.get_xy_position()}")
        self._setup(current_state)

    @abstractmethod
    def _setup(self, current_state: State):
        """ use abstraction for different setup methods. """

    def respond(self, current_state: State) -> np.ndarray:
        """ respond with input for the robot and update dashboard every second. """

        assert isinstance(current_state, State), f"current_state should be type State in type {type(current_state)}"

        system_input = self._find_input(current_state)

        self._update_prediction_error_sequence(current_state, system_input)

        # plot the controller every x seconds
        if self.dt_counter % (1/DT) == 0 and PLOT_CONTROLLER and CREATE_SERVER_DASHBOARD:
            self.update_db()
        self.dt_counter += 1

        return system_input

    @abstractmethod
    def _update_prediction_error_sequence(self, current_state: State, system_input: State):
        """ updates the sequence with the one-step-ahead prediction error. """

    @abstractmethod
    def _find_input(self, current_state: State) -> np.ndarray:
        """ find system input """
