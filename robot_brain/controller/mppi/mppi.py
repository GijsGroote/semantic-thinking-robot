import numpy as np
import torch
import pickle
import plotly.express as px
from pytorch_mppi import mppi
from robot_brain.controller.controller import Controller
from robot_brain.state import State
from robot_brain.global_variables import (
        CREATE_SERVER_DASHBOARD,
        PLOT_CONTROLLER,
        MIN_INPUT,
        MAX_INPUT,
        FIG_BG_COLOR,
        PROJECT_PATH,
        PLOT_N_TIMESTEPS
        )


class Mppi(Controller):
    """
    Model Predictive Path Intergral (MPPI) controller.
    """
    def __init__(self, order):
        Controller.__init__(self, order)
        self.name = "MPPI"
        self.mppi = None
        self.model = None
        self.n_horizon = 5
        self.y_predicted = None
        self.pred_error = []

    def _setup(self, dyn_model, current_state):
        
        self.y_predicted = current_state
        
        d = torch.device("cpu")
        self.model = dyn_model

        def running_cost(x, u):
            """ running_cost is euclidean of distance toward target and control input. """
            return torch.subtract(x[:,0], self.target_state.pos[0])**2 +\
                    torch.subtract(x[:,1], self.target_state.pos[1])**2 +\
                    1e-4*(u[:,0]**4 + u[:,1]**4)

        # create controller with chosen parameters
        self.controller = mppi.MPPI(dynamics=dyn_model,
                    running_cost=running_cost,
                    nx=self.order, # number of states in the system
                    noise_sigma=torch.tensor([[1,0],[0,1]], device=d, dtype=torch.double),
                    num_samples=1000, # number of rolouts
                    horizon=self.n_horizon,
                    lambda_=1e-2,
                    # device=d, 
                    u_min=torch.tensor([MIN_INPUT, MIN_INPUT], dtype=torch.double, device=d),
                    u_max=torch.tensor([MAX_INPUT, MAX_INPUT], dtype=torch.double, device=d)
                )

    def _set_target_state(self):
        # new running cost function
        def running_cost(x, u):
            return torch.subtract(x[:,0], self.target_state.pos[0])**2 +\
                    torch.subtract(x[:,1], self.target_state.pos[1])**2 +\
                    1e-5*(u[:,0]**4 + u[:,1]**4)
        # set the new running cost
        self.controller.running_cost = running_cost

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

    # private methods from this point
    def _find_input(self, current_state):
        """ calculate input to sent to the robot and keep track of the prediction error. """

        # TODO: this in not allowed, use inheiritance please
        match self.order:
            case 2:
                system_input = self.controller.command(current_state.get_xy_position()).cpu().numpy()

            case 3:
                system_input = self.controller.command(current_state.get_2d_pose()).cpu().numpy()

            case 4:
                system_input = self.controller.command(current_state.get_xy_dxdy()).cpu().numpy()

            case 6:
                system_input = self.controller.command(current_state.get_xyt_dxdydt()).cpu().numpy()

            case _: 
                raise ValueError(f"only order of 2, 3, 4 and 6 are available, not for {self.order}")



        if PLOT_CONTROLLER or CREATE_SERVER_DASHBOARD:

            # TODO: you actually want euclidean, not position_euclidean
            self.pred_error.append(self.y_predicted.position_euclidean(current_state))

            # calculate one step ahead prediction error
            self.y_predicted = self._simulate(current_state, system_input)

        return system_input 

    def _simulate(self, current_state, system_input) -> State:
        """ returns state simulated one time step info the future. """

        match self.order:
            case 2:
                xy_pos = self.model(torch.reshape(torch.Tensor(current_state.get_xy_position()), (1,2)),
                        torch.reshape(torch.Tensor(system_input), (1,2))).numpy()[0]

                return State(pos=np.array([xy_pos[0], xy_pos[1], 0]))

            case 3:
                xy_2d_pose = self.model(torch.reshape(torch.Tensor(current_state.get_2d_pose()), (1,3)),
                    torch.reshape(torch.Tensor(system_input), (1,2))).numpy()[0]

                return State(pos=np.array([xy_2d_pose[0], xy_2d_pose[1], 0]),
                        ang_p=np.array([0, 0, xy_2d_pose[2]]))

            case 4:
                xy_dxdy = self.model(torch.reshape(torch.Tensor(current_state.get_xy_dxdy()), (1, 4)),
                    torch.reshape(torch.Tensor(system_input), (1,2))).numpy()[0]


                return State(pos=np.array([xy_dxdy[0], xy_dxdy[1], 0]),
                        vel=np.array([xy_dxdy[2], xy_dxdy[3], 0]))
            case 6:
                xyt_dxdydt = self.model(torch.reshape(torch.Tensor(current_state.get_xyt_dxdydt()), (1,6)),
                    torch.reshape(torch.Tensor(system_input), (1,2))).numpy()[0]

                return State(pos=np.array([xyt_dxdydt[0], xyt_dxdydt[1], 0]),
                        ang_p=np.array([0, 0, xyt_dxdydt[2]]),
                        vel=np.array([xyt_dxdydt[3], xyt_dxdydt[4], 0]),
                        ang_v=np.array([0, 0, xyt_dxdydt[5]]))

            case _: 
                raise ValueError(f"only system order of 2, 3, 4 and 6 are available, not for {self.robot_order}")

    def _update_db(self):
        """ update the dashboard. """
        self.visualise(save=True)
