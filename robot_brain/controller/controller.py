from abc import ABC, abstractmethod
import numpy as np
from robot_brain.global_variables import (
        PLOT_CONTROLLER,
        CREATE_SERVER_DASHBOARD,
        DT,
        LOG_METRICS,
        )
from robot_brain.state import State

class Controller(ABC):
    """
    Controller class provides various single- and multi-body
    controllers, which calculate input for the robot.
    """

    def __init__(self, order: int):
        self.order = order
        self.dt_counter = 0

        def emptyfunction():
            pass
        self.dyn_model = emptyfunction
        self.target_state = State()
        self.pred_error = []
    
    def setup(self, dyn_model, current_state: State, target_state: State):
        """ setup the controller, this is seperated from __init__
        because dynamic models could not yet exist. """
        self.dyn_model = dyn_model
        self.target_state = target_state
        self._setup(dyn_model, current_state)

    @abstractmethod
    def _setup(self, model, current_state: State):
        """ use abstraction for different setup methods. """
        pass

    def respond(self, current_state: State) -> np.ndarray:
        """ respond with input for the robot and update dashboard every second. """

        assert isinstance(current_state, State), f"current_state should be type State in type {type(current_state)}"

        system_input = self._find_input(current_state)

        if CREATE_SERVER_DASHBOARD or PLOT_CONTROLLER or LOG_METRICS:
            
            self._update_prediction_error_sequence(current_state, system_input)

            # plot the controller every x seconds
            if self.dt_counter % (1/DT) == 0 and PLOT_CONTROLLER:# and self.dt_counter != 0:
                self._update_db()
            self.dt_counter += 1

        return system_input 

    @abstractmethod
    def _update_prediction_error_sequence(self, current_state: State, system_input: State):
        """ updates the sequence with the one-step-ahead prediction error. """
        pass

    def set_target_state(self, target_state: State):
        """ update controller to steer toward a new target state. """
        self.target_state = target_state
        self._set_target_state()

    @abstractmethod
    def _set_target_state(self):
        pass

    @abstractmethod
    def _find_input(self, current_state: State):
        pass

    def _update_db(self):
        self.visualise(save=True)
    
    @abstractmethod
    def visualise(self, save=True):
        pass

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
    def dyn_model(self):
        return self._dyn_model

    @dyn_model.setter
    # TODO: if a dyn model get some shape, update assertions for the dyn_model setter
    def dyn_model(self, val):
        assert (callable(val)), f"dyn_model must be a callable function and is {type(val)}"
        self._dyn_model = val

    @property
    def target_state(self) -> State:
        return self._target_state

    @target_state.setter
    def target_state(self, target_state: State):
        assert isinstance(target_state, State), f" target_state should be a State and is {type(target_state)}" 
        self._target_state = target_state
  
