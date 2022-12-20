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

class PushController(Controller):
    """
    Controller class for pushing controllers.
    """

    def __init__(self, order: int):
        self.order = order
        self.dt_counter = 0

        def emptyfunction():
            pass
        self.dyn_model = emptyfunction
        self.target_state = State()
        self.pred_error = []
    
    def setup(self, dyn_model, robot_state: State, obstacle_state: State, target_state: State):
        """ setup the controller, this is seperated from __init__
        because dynamic models could not yet exist. """
        self.dyn_model = dyn_model
        self.target_state = target_state
        self._setup(dyn_model, robot_state, obstacle_state)

    @abstractmethod
    def _setup(self, model, robot_state: State, obstacle_state: State):
        """ use abstraction for different setup methods. """
        pass

    def respond(self, robot_state: State, obstacle_state: State) -> np.ndarray:
        """ respond with input for the robot and update dashboard every second. """

        assert isinstance(robot_state, State), f"robot_state should be type State in type {type(current_state)}"
        assert isinstance(obstacle_state, State), f"obstacle_state should be type State in type {type(current_state)}"

        system_input = self._find_input(robot_state, obstacle_state)

        if CREATE_SERVER_DASHBOARD or PLOT_CONTROLLER or LOG_METRICS:
            
            self._update_prediction_error_sequence(robot_state, obstacle_state, system_input)

            # plot the controller every x seconds
            if self.dt_counter % (1/DT) == 0 and PLOT_CONTROLLER:# and self.dt_counter != 0:
                self.update_db()
            self.dt_counter += 1

        return system_input 

    @abstractmethod
    def _update_prediction_error_sequence(self, robot_state: State, obstacle_state: State, system_input: State):
        """ updates the sequence with the one-step-ahead prediction error. """
        pass

    @abstractmethod
    def _find_input(self, robot_state: State, obstacle_state: State) -> np.ndarray:
        pass
