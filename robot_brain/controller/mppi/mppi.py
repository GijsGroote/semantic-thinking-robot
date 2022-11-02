import numpy as np
import torch
import pickle
import plotly.express as px
from pytorch_mppi import mppi
from robot_brain.controller.controller import Controller
from robot_brain.state import State
from robot_brain.global_variables import (
        MIN_INPUT,
        MAX_INPUT,
        FIG_BG_COLOR,
        PROJECT_PATH,
        PLOT_N_TIMESTEPS
        )

from abc import abstractmethod

class Mppi(Controller):
    """
    Model Predictive Path Intergral (MPPI) controller.
    """
    def __init__(self, order: int):
        Controller.__init__(self, order)
        self.name = "MPPI"
        self.mppi = None
        self.n_horizon = 10
        self.plot_data = {}

    def _setup(self, dyn_model, current_state: State):
        """ setup the mppi controller. """

        self.y_predicted = current_state
        d = torch.device("cpu")
        self.dyn_model = dyn_model

        # create controller with chosen parameters
        self.controller = mppi.MPPI(dynamics=dyn_model,
                    running_cost=self._running_cost,
                    nx=self.order, # number of states in the system
                    noise_sigma=torch.tensor([[1,0],[0,1]], device=d, dtype=torch.double),
                    num_samples=1000, # number of rollouts
                    horizon=self.n_horizon,
                    lambda_=1e-2,
                    device=d, 
                    u_min=torch.tensor([MIN_INPUT, MIN_INPUT], dtype=torch.double, device=d),
                    u_max=torch.tensor([MAX_INPUT, MAX_INPUT], dtype=torch.double, device=d)
                )

    def _update_prediction_error_sequence(self, current_state: State, system_input: State):
        """ update prediction error and other plot data. """

        self.pred_error.append(self._calculate_prediction_error(current_state))
        self.y_predicted = self._simulate(current_state, system_input)

        # update plot data
        # TODO update plot data, do this when implementing a nicer plot

    @abstractmethod
    def _calculate_prediction_error(self, current_state: State) -> float:
        pass

    @abstractmethod
    def _simulate(self, current_state: State, system_input: State) -> State:
        pass

    def _set_target_state(self):
        self.controller.running_cost = self._running_cost 

    @abstractmethod 
    def _running_cost(self, x: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        pass
    
    def visualise(self, save=True):
        """ create plot for the mppi controller. """
        
        dt_counter = len(self.pred_error)-1

        # plot only last PLOT_N_TIMESTEPS data points
        if dt_counter >= PLOT_N_TIMESTEPS:
            time = np.arange(dt_counter-PLOT_N_TIMESTEPS, dt_counter-1, 1)
            pred_error = self.pred_error[-PLOT_N_TIMESTEPS:-1]

        else:
            time = np.arange(0, dt_counter+1, 1)
            pred_error = self.pred_error
        
        # prediction error plot
        fig = px.line(
            x=time,
            y=pred_error,
        )
        fig.update_traces(line_color='red')

        # scale the axis
        fig.update_xaxes(range=[time[0], max(time[-1], PLOT_N_TIMESTEPS)],
                         title_text="Time [steps]")

        fig.update_yaxes(range=[0, max(pred_error) * 1.2],
                         title_text="error")


        fig.update_layout({"title": {"text": "MPPI controller prediction error"}}, paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

        if save:
            # TODO: create a relative path
            with open(PROJECT_PATH+"/dashboard/data/controller.pickle", "wb") as file:
                pickle.dump(fig, file)
        else:
            fig.show()

    @property
    def y_predicted(self) -> State:
        return self._y_predicted

    @y_predicted.setter
    def y_predicted(self, val: State):
        assert isinstance(val, State), f"y_predicted should be a State and is {type(val)}"
        self._y_predicted= val
