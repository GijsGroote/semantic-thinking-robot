import numpy as np
from robot_brain.controller.mpc.template_model import template_model
from robot_brain.controller.mpc.template_mpc import template_mpc
from robot_brain.controller.mpc.template_plotter import Plotter
from robot_brain.global_variables import PLOT_CONTROLLER, CREATE_SERVER_DASHBOARD, DT
import torch
from pytorch_mppi import mppi
from casadi import vertcat
from robot_brain.controller.controller import Controller

class Mppi(Controller):
    """
    Model Predictive Path Intergral (MPPI) controller.
    """
    def __init__(self):
        Controller.__init__(self)
        self.mppi = None
        self.model = None
        self.target_state = None
        self.n_horizon = 5
        self.y_predicted = None
        self.pred_error = []

    def setup(self, dyn_model, current_state, target_state):

        d = torch.device("cpu")

        def running_cost(x, u):
            x = x.detach().numpy()
            # loop over all rollouts
            next_state = np.zeros(x.shape)
            cost = np.zeros(next_state.shape[0])
            for k in range(next_state.shape[0]):
                next_state[k,:] = np.array([x[k,0] + 0.05 *  u[k,0],
                    x[k,1] + 0.05 *  u[k,1],
                    x[k,2]]
                    ).reshape((3))
                 # linear cost toward the target
                x_cost = abs(target_state.pos[0] - next_state[k,0])
                y_cost = abs(target_state.pos[1] - next_state[k,1])
                t_cost = abs(target_state.ang_p[2] - next_state[k,2])

                cost[k] = x_cost + y_cost + t_cost
                
                # print(f"the next states {next_state}")

            cost = torch.tensor(cost)
            # print(f'retrunign cost {cost.shape}')

            return cost

        # create controller with hosen parameters
        self.controller = mppi.MPPI(dynamics=dyn_model,
                running_cost=running_cost,
                nx=3, # number of states in the system
                noise_sigma=torch.tensor([[1.5,0],[0,1.5]], device=d, dtype=torch.double),
                num_samples=250, # number of rolouts
                horizon=self.n_horizon,
                lambda_=1e-2,
                # device=d, 
                u_min=torch.tensor([-1.5, -1.5], dtype=torch.double, device=d),
                u_max=torch.tensor([1.5, 1.5], dtype=torch.double, device=d)
                )

    def find_input(self, current_state):

        action = self.controller.command(current_state.get_2d_pose()).cpu().numpy()

        if PLOT_CONTROLLER or CREATE_SERVER_DASHBOARD:
            pass
            
        return action

    def set_target_state(self, state):


        def running_cost(x, u):
            x = x.detach().numpy()
            # loop over all rollouts
            next_state = np.zeros(x.shape)
            cost = np.zeros(next_state.shape[0])
            for k in range(next_state.shape[0]):
                next_state[k,:] = np.array([x[k,0] + 0.05 *  u[k,0],
                    x[k,1] + 0.05 *  u[k,1],
                    x[k,2]]
                    ).reshape((3))
                 # linear cost toward the target

                x_cost = abs(state.pos[0] - next_state[k,0])
                y_cost = abs(state.pos[1] - next_state[k,1])
                t_cost = abs(state.ang_p[2] - next_state[k,2])

                cost[k] = x_cost + y_cost + t_cost
                
                # print(f"the next states {next_state}")

            cost = torch.tensor(cost)
            # print(f'retrunign cost {cost.shape}')

            return cost


        self.controller.running_cost = running_cost

    def visualise(self):
        pass

    def update_db(self):
        pass
