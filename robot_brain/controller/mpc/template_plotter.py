import numpy as np
import os
import pickle
from numpy import dstack
from plotly.subplots import make_subplots
from pyarrow import feather
import plotly.graph_objects as go
import pandas as pd
from robot_brain.global_variables import FIG_BG_COLOR, DT, PROJECT_PATH, PLOT_N_TIMESTEPS
pd.options.plotting.backend = "plotly"

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
            x=[time[0], time[-1]],
            y=x_ref*np.ones((2,)),
            name="x-ref",
            line=dict(color='medium purple', width=1, dash='dash')
            ), row=1, col=1)
        fig.append_trace(go.Scatter(
            x=[time[0], time[-1]],
            y=y_ref*np.ones((2,)),
            name="y-ref",
            line=dict(color='forest green', width=1, dash='dash')
            ), row=1, col=1)
        fig.append_trace(go.Scatter(
            x=[time[0], time[-1]],
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
        fig.update_xaxes(range=[time[0], max(time[-1], PLOT_N_TIMESTEPS)],
                         row=1, col=1)

        fig.update_xaxes(range=[time[0], max(time[-1], PLOT_N_TIMESTEPS)],
                         title_text="Time [steps]",
                         row=2, col=1)

        fig.update_yaxes(range=[dstack((x_pos, y_pos, theta)).min() - 0.2,
                                dstack((x_pos, y_pos, theta)).max() + 0.2],
                         title_text="position",
                         row=1, col=1)

        fig.update_yaxes(range=[dstack((sys_input1, sys_input2)).min() - 0.2,
                                dstack((sys_input1, sys_input2)).max() + 0.2],
                         title_text="input & error",
                         row=2, col=1)

        fig.update_layout({"title": {"text": "MPC controller"}})

        fig.update_layout(paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

        if save:
            with open(PROJECT_PATH+"/dashboard/data/controller.pickle", "wb") as file:
                pickle.dump(fig, file)
        else:
            fig.show()

    def update_db(self, target_state, pred_error):
        """ update the dashboard. """
        self.visualise(target_state, pred_error, save=True)
   
