import do_mpc
import numpy as np
from robot_brain.controller.mpc.template_model import template_model
from robot_brain.controller.mpc.template_mpc import template_mpc
from robot_brain.controller.mpc.template_plotter import Plotter
from robot_brain.global_variables import PLOT_CONTROLLER, CREATE_SERVER_DASHBOARD, DT
from robot_brain.controller.controller import Controller

class Mpc(Controller):
    """
    Model Predictive Control controller, finds the optimal input that steers
    the system toward the target state by minimizing an objective function.
    """
    def __init__(self):
        Controller.__init__(self)
        self.mpc = None
        self.model = None
        self.simulator = None
        self.plotter = None
        self.target_state = None
        self.n_horizon = 15
        self.y_predicted = None
        self.pred_error = []


    def setup(self, dyn_model, current_state, target_state):
        self.target_state = target_state

        # fully define model
        self.model = template_model(dyn_model)

        # set all mpc parameters
        self.mpc = template_mpc(self.model, 15, target_state)
        initial_state = np.array([
            current_state.pos[0],
            current_state.pos[0],
            current_state.ang_p[2]])
        self.mpc.x0 = initial_state

        self.mpc.set_initial_guess()

        if PLOT_CONTROLLER or CREATE_SERVER_DASHBOARD:

            simulator = do_mpc.simulator.Simulator(self.model)
            tvp_template = simulator.get_tvp_template()

            def tvp_fun(t_now): # pylint: disable=unused-argument
                tvp_template['pos_x_target'] = target_state.pos[0]
                tvp_template['pos_y_target'] = target_state.pos[1]
                tvp_template['ang_p_target'] = target_state.ang_p[2]

                return tvp_template

            simulator.set_tvp_fun(tvp_fun)

            # Set parameter(s):
            simulator.set_param(t_step=DT)
            simulator.setup()

            simulator.x0 = initial_state

            self.simulator = simulator
            self.plotter = Plotter()
            self.plotter.setup(self.mpc, self.simulator)

            self.y_predicted = current_state.get_2d_pose()

        self.mpc.reset_history()

    def find_input(self, current_state):
        initial_state = current_state.get_2d_pose()
        self.mpc.x0 = initial_state
        system_input = self.mpc.make_step(initial_state) # solves minimisation problem

        if PLOT_CONTROLLER or CREATE_SERVER_DASHBOARD:
            # calculate one step ahead prediction error
            self.pred_error.append(np.linalg.norm(self.y_predicted - initial_state))

            self.simulator.x0 = initial_state
            self.y_predicted = np.reshape(
                    self.simulator.make_step(system_input), initial_state.shape)

        return np.reshape(system_input, (len(system_input),))

    def set_target_state(self, state):

        self.target_state = state

        tvp_template = self.mpc.get_tvp_template()

        def tvp_fun(t_now): # pylint: disable=unused-argument
            for k in range(self.n_horizon+1):
                tvp_template['_tvp',k,'pos_x_target'] = state.pos[0]
                tvp_template['_tvp',k,'pos_y_target'] = state.pos[1]
                tvp_template['_tvp',k,'ang_p_target'] = state.ang_p[2]

            return tvp_template

        self.mpc.set_tvp_fun(tvp_fun)


    def visualise(self):
        self.plotter.visualise(self.target_state, self.pred_error)

    def update_db(self):
        self.plotter.update_db(self.target_state, self.pred_error)
