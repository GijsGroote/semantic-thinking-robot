import do_mpc
import numpy as np
from numpy import dstack
import pickle
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import pandas as pd
from robot_brain.global_variables import FIG_BG_COLOR, DT, PROJECT_PATH, PLOT_N_TIMESTEPS
pd.options.plotting.backend = "plotly"

from robot_brain.controller.drive.mpc.mpc import Mpc
from robot_brain.state import State
from robot_brain.global_variables import PLOT_CONTROLLER, DT, MAX_INPUT, MIN_INPUT

class Plotter():
    """
    Plot/store the mpc data.
    """
    def __init__(self):
        self.simulator = None
        self.mpc = None

    def setup(self, mpc, simulator):
        self.mpc = mpc
        self.simulator = simulator

    def visualise(self, target_state, pred_error, save=True):
        """ Visualise the MPC controller. """

        x_ref= target_state.pos[0]
        y_ref= target_state.pos[1]
        theta_ref =  target_state.ang_p[2]
        # TODO: loop only through the last PLOT_N_TIMESTEPS of self.mpc.data
        x_pos = [i[0] for i in self.mpc.data["_x"]]
        y_pos = [i[1] for i in self.mpc.data["_x"]]
        theta = [i[2] for i in self.mpc.data["_x"]]
        sys_input1 = [i[0] for i in self.mpc.data['_u']]
        sys_input2 = [i[1] for i in self.mpc.data['_u']]

        dt_counter = len(x_pos)

        # plot only last PLOT_N_TIMESTEPS data points
        if dt_counter >= PLOT_N_TIMESTEPS:
            time = np.arange(dt_counter-PLOT_N_TIMESTEPS, dt_counter, 1)
            x_pos = x_pos[-PLOT_N_TIMESTEPS:-1]
            y_pos = y_pos[-PLOT_N_TIMESTEPS:-1]
            theta = theta[-PLOT_N_TIMESTEPS:-1]
            sys_input1 = sys_input1[-PLOT_N_TIMESTEPS:-1]
            sys_input2 = sys_input2[-PLOT_N_TIMESTEPS:-1]
            pred_error = pred_error[-PLOT_N_TIMESTEPS: -1]

        else:
            time = np.arange(0, dt_counter, 1)

        fig = make_subplots(rows=2, cols=1)

        # x, y and theta positions
        fig.append_trace(go.Scatter(
            x=time,
            y=x_pos,
            name="x-position",
            line=dict(color='medium purple')
            ), row=1, col=1)
        fig.append_trace(go.Scatter(
            x=time,
            y=y_pos,
            name="y-position",
            line=dict(color='forest  green')
            ), row=1, col=1)
        fig.append_trace(go.Scatter(
            x=time,
            y=theta,
            name="orientation",
            line=dict(color='dark green')
            ), row=1, col=1)

        # reference signals
        fig.append_trace(go.Scatter(
            x=[time[0], time[0]+PLOT_N_TIMESTEPS],
            y=x_ref*np.ones((2,)),
            name="x-ref",
            line=dict(color='medium purple', width=1, dash='dash')
            ), row=1, col=1)
        fig.append_trace(go.Scatter(
            x=[time[0], time[0]+PLOT_N_TIMESTEPS],
            y=y_ref*np.ones((2,)),
            name="y-ref",
            line=dict(color='forest green', width=1, dash='dash')
            ), row=1, col=1)
        fig.append_trace(go.Scatter(
            x=[time[0], time[0]+PLOT_N_TIMESTEPS],
            y=theta_ref*np.ones((2,)),
            name="orien-ref",
            line=dict(color='dark green', width=1, dash='dash')
            ), row=1, col=1)

        # input
        fig.append_trace(go.Scatter(
            x=time,
            y=sys_input1,
            name="u1",
            line=dict(color='silver', shape='hv')
        ), row=2, col=1)
        fig.append_trace(go.Scatter(
            x=time,
            y=sys_input2,
            name="u2",
            line=dict(color='gray', shape='hv'),
        ), row=2, col=1)

        # prediction error plot
        fig.append_trace(go.Scatter(
            x=time,
            y=pred_error,
            name="predict error",
            line=dict(color='red'),
        ), row=2, col=1)

        # scale the axis
        fig.update_xaxes(range=[time[0], max(time[0]+PLOT_N_TIMESTEPS, PLOT_N_TIMESTEPS)],
                         row=1, col=1)

        fig.update_xaxes(range=[time[0], max(time[0]+PLOT_N_TIMESTEPS, PLOT_N_TIMESTEPS)],
                         title_text="Time [steps]",
                         row=2, col=1)

        fig.update_yaxes(range=[dstack((x_pos, y_pos, theta)).min() - 1.5,
                                dstack((x_pos, y_pos, theta)).max() + 1.5],
                         title_text="position",
                         row=1, col=1)

        fig.update_yaxes(range=[min(MIN_INPUT, min(pred_error)) - 0.2,
                                max(MAX_INPUT, max(pred_error)) + 0.2],
                         title_text="input & error",
                         row=2, col=1)

        fig.update_layout({"title": {"text": "MPC controller"}})

        fig.update_layout(paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

        if save:
            with open(PROJECT_PATH+"dashboard/data/cntrl.pickle", "wb") as file:
                pickle.dump(fig, file)
        else:
            fig.show()
 
class DriveMpc3thOrder(Mpc):
    """
    Model Predictive Control controller for a 3th order system model. 
    Such as the boxer robot with velocity input.
    """
    def __init__(self):
        Mpc.__init__(self, order=3)

    def _set_target_state(self):
        tvp_template = self.mpc.get_tvp_template()

        def tvp_fun(t_now): # pylint: disable=unused-argument
            for k in range(self.n_horizon+1):
                tvp_template['_tvp',k,'pos_x_target'] = self.target_state.pos[0]
                tvp_template['_tvp',k,'pos_y_target'] = self.target_state.pos[1]
                tvp_template['_tvp',k,'ang_p_target'] = self.target_state.ang_p[2]

            return tvp_template

        self.mpc.set_tvp_fun(tvp_fun)

    def create_initial_state(self, current_state: State) -> np.ndarray:
        return np.array([
            current_state.pos[0],
            current_state.pos[1],
            current_state.ang_p[2]])

    def create_tvp_sim(self):
        """ return template for time-varying parameters. """

        tvp_template = self.simulator.get_tvp_template()

        def tvp_fun(t_now): # pylint: disable=unused-argument
            tvp_template['pos_x_target'] = self.target_state.pos[0]
            tvp_template['pos_y_target'] = self.target_state.pos[1]
            tvp_template['ang_p_target'] = self.target_state.ang_p[2]

            return tvp_template

        return tvp_fun

    def _find_input(self, current_state: State) -> np.ndarray:
        """ solves minimisation problem. """
        initial_state = current_state.get_2d_pose()
        self.mpc.x0 = initial_state
        system_input = self.mpc.make_step(initial_state) 

        return np.reshape(system_input, (len(system_input),))

    def calculate_prediction_error(self, current_state: State) -> float:
        """ return calculated prediction error. """
        return self.y_predicted.pose_euclidean(current_state)

    def simulate(self, system_input: np.ndarray) -> State:
        """ return simulated state one step into the future. """
        pred_output = self.simulator.make_step(system_input)
        return State(pos=np.array([pred_output[0].item(), pred_output[1].item(), 0]),
            ang_p=np.array([0, 0, pred_output[2].item()]))

    def template_model(self, dyn_model):
        # Obtain an instance of the do-mpc model class
        model_type = 'discrete'     # either 'discrete' or 'continuous'
        model = do_mpc.model.Model(model_type)

        # state variables
        pos_x = model.set_variable(var_type='_x', var_name='pos_x', shape=(1, 1))
        pos_y = model.set_variable(var_type='_x', var_name='pos_y', shape=(1, 1))
        ang_p = model.set_variable(var_type='_x', var_name='ang_p', shape=(1, 1))

        # merged state variables
        # dx = model.set_variable(var_type='_x', var_name='dx', shape=(3, 1))

        # inputs
        sys_input1 = model.set_variable(var_type='_u', var_name='u1', shape=(1, 1))
        sys_input2 = model.set_variable(var_type='_u', var_name='u2', shape=(1, 1))

        # create time varying target parameters
        model.set_variable(var_type='_tvp', var_name='pos_x_target', shape=(1, 1))
        model.set_variable(var_type='_tvp', var_name='pos_y_target', shape=(1, 1))
        model.set_variable(var_type='_tvp', var_name='ang_p_target', shape=(1, 1))

        # changing parameters, moment of inertia or mass
        # mass = model.set_variable('parameter', 'mass')

        # set right hand site, todo: this should come from the dynamics model
        model.set_rhs('pos_x', pos_x)
        model.set_rhs('pos_y', pos_y)
        model.set_rhs('ang_p', ang_p)

        # right hand side equation f(x)
        dx_next = dyn_model([pos_x, pos_y, ang_p], [sys_input1, sys_input2])

        model.set_rhs('pos_x', dx_next[0,:])
        model.set_rhs('pos_y', dx_next[1,:])
        model.set_rhs('ang_p', dx_next[2,:])

        # model.set_rhs('pos_x', pos_x + 0.05*np.cos(ang_p) * u1)
        # model.set_rhs('pos_y', pos_y + 0.05*np.sin(ang_p) * u1)
        # model.set_rhs('ang_p', ang_p + 0.05 * u2)

        model.setup()

        return model

    def template_mpc(self, model, n_horizon, target_state):

        # Obtain an instance of the do-mpc MPC class
        mpc = do_mpc.controller.MPC(model)

        # suppress output
        suppress_ipopt = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
        mpc.set_param(nlpsol_opts=suppress_ipopt)

        # Set parameters:
        setup_mpc = {
            'n_horizon': n_horizon,
            't_step': DT,
            'n_robust': 1,
            'store_full_solution': PLOT_CONTROLLER,
        }

        rterm_u1 = 1e-2
        rterm_u2 = 1e-2
        mpc.set_param(**setup_mpc)


        lterm = (model.x["pos_x"]-model.tvp["pos_x_target"]) ** 2 + \
        (model.x["pos_y"]-model.tvp["pos_y_target"]) ** 2 + \
        0.1*(model.x["ang_p"]-model.tvp["ang_p_target"]) ** 2
        mterm = 0.5*lterm

        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(
            u1=rterm_u1,
            u2=rterm_u2
        )

        tvp_template = mpc.get_tvp_template()
        def tvp_fun(t_now): # pylint: disable=unused-argument
            for k in range(n_horizon+1):
                tvp_template['_tvp',k,'pos_x_target'] = target_state.pos[0]
                tvp_template['_tvp',k,'pos_y_target'] = target_state.pos[1]
                tvp_template['_tvp',k,'ang_p_target'] = target_state.ang_p[2]

            return tvp_template

        mpc.set_tvp_fun(tvp_fun)

        # Lower bounds on states:
        # mpc.bounds['lower', '_x', 'ang_p'] = -2 * np.pi
        # Upper bounds on states
        # mpc.bounds['upper', '_x', 'ang_p'] = 2 * np.pi

        # Lower bounds on inputs:
        mpc.bounds['lower', '_u', 'u1'] = MIN_INPUT
        mpc.bounds['lower', '_u', 'u2'] = MIN_INPUT
        # upper bounds on inputs:
        mpc.bounds['upper', '_u', 'u1'] = MAX_INPUT
        mpc.bounds['upper', '_u', 'u2'] = MAX_INPUT

        mpc.setup()

        return mpc

    def create_plotter(self) -> Plotter:
        """ return a plotter object to visualise what the mpc is doing. """
        plotter = Plotter()
        plotter.setup(self.mpc, self.simulator)
        return plotter
