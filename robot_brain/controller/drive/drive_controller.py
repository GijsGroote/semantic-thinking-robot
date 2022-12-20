from abc import abstractmethod
import numpy as np
from robot_brain.global_variables import (
        PLOT_CONTROLLER,
        CREATE_SERVER_DASHBOARD,
        DT,
        LOG_METRICS,
        )
from robot_brain.state import State
from robot_brain.controller.controller import Controller

class DriveController(Controller):
    """
    Controller class for driving.
    """

    def __init__(self, order: int):
        Controller.__init__(self, order)
    
    def respond(self, current_state: State) -> np.ndarray:
        """ respond with input for the robot and update dashboard every second. """

        assert isinstance(current_state, State), f"current_state should be type State in type {type(current_state)}"

        system_input = self._find_input(current_state)

        if CREATE_SERVER_DASHBOARD or PLOT_CONTROLLER or LOG_METRICS:
            
            self._update_prediction_error_sequence(current_state, system_input)

            # plot the controller every x seconds
            if self.dt_counter % (1/DT) == 0 and PLOT_CONTROLLER:
                self.update_db()
            self.dt_counter += 1

        return system_input 

    @abstractmethod
    def _update_prediction_error_sequence(self, current_state: State, system_input: State):
        """ updates the sequence with the one-step-ahead prediction error. """
        pass

    @abstractmethod
    def _find_input(self, current_state: State) -> np.ndarray:
        pass
