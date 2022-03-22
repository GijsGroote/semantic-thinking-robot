from dash import Dash, html, dcc
from dash.dependencies import Input, Output
import multiprocessing
import pandas as pd
from os.path import exists

pd.options.plotting.backend = "plotly"
from plotly.subplots import make_subplots
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
from pathlib import Path
import pyarrow.feather as feather
from numpy import dstack


class Dashboard:

    def __init__(self):
        self.app = Dash(__name__, title="DashBoard", update_title="DashBoard")
        self.no_data_found = {
            "layout": {
                "paper_bgcolor": "#e5ecf6",
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

        self.app.layout = html.Div(children=[
            html.Div([
                html.Div([
                    html.H4('HGraph'),
                    dcc.Graph(id='live-update-hgraph', animate=False),
                    dcc.Interval(
                        id='hgraph-interval-component',
                        interval=1 * 1000,  # in milliseconds
                        n_intervals=0
                    )
                ], className="item"),
                html.Div(className="item-spacer"),
                html.Div([
                    html.H4('Controller Live Feed'),
                    html.Div(id='live-update-text'),
                    dcc.Graph(id='live-update-controller-graph', animate=True),
                    dcc.Interval(
                        id='controller-interval-component',
                        interval=1 * 1000,  # in milliseconds
                        n_intervals=0
                    )
                ], className="item"),
                html.Div([
                    html.H4('KGraph, todo'),
                    dcc.Graph(figure=self.no_data_found)
                ], className="item"),
                html.Div(className="item-spacer"),
                html.Div([
                    html.H4('extra plot space'),
                    dcc.Graph(figure=self.no_data_found)
                ], className="item"),

            ], className="container")
        ])

        @self.app.callback(Output('live-update-hgraph', 'figure'),
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

        @self.app.callback(Output('live-update-text', 'children'),
                           Input('interval-component', 'n_intervals'))
        def update_metrics(n):
            return [
                html.Span('n: {0:.2f}'.format(n))
            ]

        @self.app.callback(Output('live-update-controller-graph', 'figure'),
                           Input('controller-interval-component', 'n_intervals'))
        def update_controller_graph_live(n):

            # read in controller data if it exists
            if Path("../dashboard/mpc_data.feather").is_file():
                df = feather.read_feather("../dashboard/mpc_data.feather")
            else:
                return self.no_data_found

            # todo: this can be done better, send metadata with the dataframe
            if df.type[0] == "mpc":
                fig = make_subplots(rows=2, cols=1)
                # px.line(df, x="x", y="y", title="Unsorted Input")
                fig.append_trace(go.Scatter(
                    x=df["time"],
                    y=df["x"],
                    name="x position"
                ), row=1, col=1)
                fig.append_trace(go.Scatter(
                    x=df["time"],
                    y=df["y"],
                    name="y position"
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

                max_bound = max(15, df["time"][df.index[-1]] + 1)
                fig.update_xaxes(range=[df["time"][0], max_bound])
                fig.update_yaxes(range=[dstack((df["x"], df["y"], df["theta"])).max() + 0.2,
                                        dstack((df["x"], df["y"], df["theta"])).min() - 0.2])

                return fig
            else:
                return self.no_data_found

    def startDashServer(self):
        def run():
            self.app.scripts.config.serve_locally = True
            self.app.run_server(
                port=8050,
                debug=False,
                processes=4,
                threaded=False
            )

        # Run on a separate process so that it doesn't block
        self.app.server_process = multiprocessing.Process(target=run)
        self.app.server_process.start()
