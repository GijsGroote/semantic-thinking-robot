from abc import abstractmethod
import do_mpc
from robot_brain.state import State
from robot_brain.system_model import SystemModel
import numpy as np

from robot_brain.global_variables import (
        PLOT_CONTROLLER,
        CREATE_SERVER_DASHBOARD,
        DT,
        LOG_METRICS,
        )
from robot_brain.controller.drive.drive_controller import DriveController

class Mpc(DriveController):
    """
    Model Predictive Control controller, finds the optimal input that steers
    the system toward the target state by minimizing an objective function.
    """
    def __init__(self, order):
        # TODO: this class, and child classes use the simulator for estimating the next state,
        # an improvement would be to use the estimator.
        DriveController.__init__(self, order)
        self.name = "MPC"
        self.mpc = None
        self.mpc_model = None
        self.simulator = None
        self.plotter = None
        self.n_horizon = 15

    def _setup(self, current_state: State):
        # fully define model
        self.mpc_model = self.template_model(self.system_model.model)

        # set all mpc parameters
        self.mpc = self.template_mpc(model=self.mpc_model,
                n_horizon=self.n_horizon,
                target_state=self.target_state)

        initial_state = self.create_initial_state(current_state)
        self.mpc.x0 = initial_state
        self.mpc.set_initial_guess()

        self.simulator = do_mpc.simulator.Simulator(self.mpc_model)
        self.simulator.set_tvp_fun(self.create_tvp_sim())

        # Set parameter(s):
        self.simulator.set_param(t_step=DT)
        self.simulator.setup()

        self.simulator.x0 = initial_state

        self.y_predicted = current_state

        if PLOT_CONTROLLER or CREATE_SERVER_DASHBOARD or LOG_METRICS:
            self.plotter = self.create_plotter()


        self.mpc.reset_history()

    @abstractmethod
    def template_model(self, dyn_model):
        "todo"

    @abstractmethod
    def template_mpc(self):
        "todo"

    @abstractmethod
    def create_plotter(self):
        "todo"

    @abstractmethod
    def create_tvp_sim(self):
        "todo"

    def _update_prediction_error_sequence(self, current_state: State, system_input: np.ndarray):
        """ update the prediction error and calculate the one step ahead prediction. """
        system_input = np.reshape(system_input, (system_input.shape[0], 1))
        self.pred_error.append(self.calculate_prediction_error(current_state))
        self.simulator.x0 = self.create_initial_state(current_state)
        self.y_predicted = self.simulate(system_input)

    @abstractmethod
    def simulate(self, system_input: np.ndarray) -> State:
        """TODO"""

    @abstractmethod
    def calculate_prediction_error(self, current_state: State) -> float:
        """TODO"""

    def visualise(self, save=True):
        self.plotter.visualise(self.target_state, self.pred_error, save=save)
