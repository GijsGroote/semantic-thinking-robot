from dash import Dash, html, dcc
from dash.dependencies import Input, Output
import multiprocessing
import pandas as pd

pd.options.plotting.backend = "plotly"
from plotly.subplots import make_subplots
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
from pathlib import Path
import os
import pyarrow.feather as feather
from numpy import dstack

from robot_brain.graphs.Edge import Edge

from robot_brain.graphs.ConfSetNode import ConfSetNode

from robot_brain.graphs.ObjectSetNode import ObjectSetNode
from robot_brain.graphs.HGraph import HGraph
from pyvis.network import Network

def register_callbacks(app):
    no_data_found = {
        "layout": {
            "paper_bgcolor": "rgb(26, 26, 26)",
            "xaxis": {
                "visible": False
            },
            "yaxis": {
                "visible": False
            },
            "annotations": [
                {
                    "text": "No matching data found",
                    "xref": "paper",
                    "yref": "paper",
                    "showarrow": False,
                    "font": {
                        "size": 28
                    }
                }
            ]
        }
    }

    @app.callback(Output('live-update-hgraph', 'figure'),
                       Input('hgraph-interval-component', 'n_intervals'))
    def update_hgraph_live(n):
        # todo: this can be done better, send metadata with the dataframe
        if True:
            # TODODOODOD
            fig = px.bar(pd.DataFrame([1, 2, 2], [3, 2, 4]))
            fig.update_layout(paper_bgcolor="#e5ecf6")
            return fig
        else:
            return self.no_data_found


    @app.callback(Output('live-update-controller-graph', 'figure'),
                       Input('controller-interval-component', 'n_intervals'))
    def update_controller_graph_live(n):
        # read in controller data if it exists
        if Path("../robot_brain/dashboard/data/mpc_data.feather").is_file():

            df = feather.read_feather("../robot_brain/dashboard/data/mpc_data.feather")
        else:
            return no_data_found

        # todo: this can be done better, send metadata with the dataframe
        if df.type[0] == "mpc":
            fig = make_subplots(rows=2, cols=1)
            # px.line(df, x="x", y="y", title="Unsorted Input")
            fig.append_trace(go.Scatter(
                x=df["time"],
                y=df["x"],
                name="x",
            ), row=1, col=1)
            fig.append_trace(go.Scatter(
                x=df["time"],
                y=df["y"],
                name="y"
            ), row=1, col=1)
            fig.append_trace(go.Scatter(
                x=df["time"],
                y=df["theta"],
                name="theta"
            ), row=1, col=1)

            fig.append_trace(go.Scatter(
                x=df["time"],
                y=df["u1"],
                name="u1"
            ), row=2, col=1)
            fig.append_trace(go.Scatter(
                x=df["time"],
                y=df["u2"],
                name="u2"
            ), row=2, col=1)
            fig.update_layout(paper_bgcolor="#e5ecf6")

            # scale the axis
            fig.update_xaxes(range=[df["time"][0], max(15, df["time"][df.index[-1]] + 1)],
                             row=1, col=1)

            fig.update_xaxes(range=[df["time"][0], max(15, df["time"][df.index[-1]] + 1)],
                             title_text="Time [sec]",
                             row=2, col=1)

            fig.update_yaxes(range=[dstack((df["x"], df["y"], df["theta"])).max() + 0.2,
                                    dstack((df["x"], df["y"], df["theta"])).min() - 0.2],
                             title_text="position [-]",
                             row=1, col=1)

            fig.update_yaxes(range=[dstack((df["u1"], df["u2"])).max() + 0.2,
                                    dstack((df["u1"], df["u2"])).min() - 0.2],
                             title_text="input [-]",
                             row=2, col=1)

            fig.update_layout({"title": {"text": "MPC controller"}})

            return fig
        else:
            return no_data_found
