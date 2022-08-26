from robot_brain.global_variables import FIG_BG_COLOR, DT

import numpy as np
from numpy import dstack
from plotly.subplots import make_subplots
from pyarrow import feather
import plotly.graph_objects as go
import pandas as pd
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

    def visualise(self, target_state, pred_error):
        x_ref= target_state.pos[0]
        y_ref= target_state.pos[1]
        theta_ref =  target_state.ang_p[2]
        x_pos = [i[0] for i in self.mpc.data["_x"]]
        y_pos = [i[1] for i in self.mpc.data["_x"]]
        theta = [i[2] for i in self.mpc.data["_x"]]
        sys_input1 = [i[0] for i in self.mpc.data['_u']]
        sys_input2 = [i[1] for i in self.mpc.data['_u']]

        time = np.arange(0, len(self.mpc.data["_x"]), 1)

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
            name="orientation-ref",
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
            name="one-step-ahead prediction error",
            line=dict(color='red'),
        ), row=2, col=1)

        # scale the axis
        fig.update_xaxes(range=[time[0], time[-1]],
                         row=1, col=1)

        fig.update_xaxes(range=[time[0], time[-1]],
                         title_text="Time [steps]",
                         row=2, col=1)

        fig.update_yaxes(range=[dstack((x_pos, y_pos, theta)).min() - 0.2,
                                dstack((x_pos, y_pos, theta)).max() + 0.2],
                         title_text="position [-]",
                         row=1, col=1)

        fig.update_yaxes(range=[dstack((sys_input1, sys_input2)).min() - 0.2,
                                dstack((sys_input1, sys_input2)).max() + 0.2],
                         title_text="input [-] & error [-]",
                         row=2, col=1)

        fig.update_layout({"title": {"text": "MPC controller"}})

        fig.update_layout(paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)
        fig.show()

    def update_db(self, target_state, pred_error):
        """
        Stores the MPC data as feather file, for the dashboard
        """

        dictionary = {
            "type": "mpc",
            "x_ref": target_state.pos[0],
            "y_ref": target_state.pos[1],
            "theta_ref": target_state.ang_p[2],
            "x": [i[0] for i in self.mpc.data["_x"]],
            "y": [i[1] for i in self.mpc.data["_x"]],
            "theta": [i[2] for i in self.mpc.data["_x"]],
            "u1": [i[0] for i in self.mpc.data['_u']],
            "u2": [i[1] for i in self.mpc.data['_u']],
            "pred_error": pred_error,
        }

        # find current time
        dt_counter = len(dictionary["x"])
        current_time = dt_counter*DT

        # TODO: metadata so I can tell this data is mpc data
        data_frame = pd.DataFrame(dictionary)

        # store only last n_horizon data points
        if current_time >= self.mpc.n_horizon:
            time = np.arange(current_time-self.mpc.n_horizon, current_time, DT)
            data_frame = data_frame.tail(len(time))
            data_frame["time"] = time
        else:
            data_frame["time"] = np.arange(data_frame.index[0], current_time, DT)

        feather.write_feather(data_frame, '../robot_brain/dashboard/data/mpc_data.feather')
