import do_mpc
import pandas as pd

pd.options.plotting.backend = "plotly"
from plotly.subplots import make_subplots
import plotly.express as px
import numpy as np
import plotly.graph_objects as go
import os
import pyarrow.feather as feather
from numpy import dstack
from robot_brain.global_variables import * 


class Plotter():

    def __init__(self):
        self.simulator = None
        self.mpc = None

    def setup(self, mpc, simulator):
        self.mpc = mpc
        self.simulator = simulator

         
    def visualise(self, targetState, predError):
        x_ref= targetState.pos[0]
        y_ref= targetState.pos[1]
        theta_ref =  targetState.ang_p[2]
        x = [i[0] for i in self.mpc.data["_x"]]
        y = [i[1] for i in self.mpc.data["_x"]]
        theta = [i[2] for i in self.mpc.data["_x"]]
        u1 = [i[0] for i in self.mpc.data['_u']]
        u2 = [i[1] for i in self.mpc.data['_u']]


        time = np.arange(0, len(self.mpc.data["_x"]), 1)

        fig = make_subplots(rows=2, cols=1)

        # x, y and theta positions
        fig.append_trace(go.Scatter(
            x=time,
            y=x,
            name="x-position",
            line=dict(color='medium purple')
            ), row=1, col=1)
        fig.append_trace(go.Scatter(
            x=time,
            y=y,
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
            y=u1,
            name="u1",
            line=dict(color='silver', shape='hv')
        ), row=2, col=1)
        fig.append_trace(go.Scatter(
            x=time,
            y=u2,
            name="u2",
            line=dict(color='gray', shape='hv'),
        ), row=2, col=1)

        # prediction error plot
        fig.append_trace(go.Scatter(
            x=time,
            y=predError,
            name="one-step-ahead prediction error",
            line=dict(color='red'),
        ), row=2, col=1)        

        # scale the axis
        fig.update_xaxes(range=[time[0], time[-1]],
                         row=1, col=1)

        fig.update_xaxes(range=[time[0], time[-1]],
                         title_text="Time [steps]",
                         row=2, col=1)

        fig.update_yaxes(range=[dstack((x, y, theta)).min() - 0.2,
                                dstack((x, y, theta)).max() + 0.2],
                         title_text="position [-]",
                         row=1, col=1)

        fig.update_yaxes(range=[dstack((u1, u2)).min() - 0.2,
                                dstack((u1, u2)).max() + 0.2],
                         title_text="input [-] & error [-]",
                         row=2, col=1)

        fig.update_layout({"title": {"text": "MPC controller"}})
        
        fig.update_layout(paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)
        
        fig.show()
        
         
    def updateDB(self, targetState, predError):
        """
        Stores the MPC data as feather file, for the dashboard
        """

        dictionary = {
            "type": "mpc",
            "x_ref": targetState.pos[0], 
            "y_ref": targetState.pos[1],
            "theta_ref": targetState.ang_p[2], 
            "x": [i[0] for i in self.mpc.data["_x"]],
            "y": [i[1] for i in self.mpc.data["_x"]],
            "theta": [i[2] for i in self.mpc.data["_x"]],
            "u1": [i[0] for i in self.mpc.data['_u']],
            "u2": [i[1] for i in self.mpc.data['_u']],
            "pred_error": predError,
        }

        # find current time
        dt_counter = len(dictionary["x"])
        currentTime = dt_counter*DT

        # todo: metadata so I can tell this data is mpc data
        df = pd.DataFrame(dictionary)

        # store only last n_horizon data points
        if currentTime >= self.mpc.n_horizon:
            time = np.arange(currentTime-self.mpc.n_horizon, currentTime, DT)
            df = df.tail(len(time))
            df["time"] = time
        else:
            df["time"] = np.arange(df.index[0], currentTime, DT)

        feather.write_feather(df, '../robot_brain/dashboard/data/mpc_data.feather')


