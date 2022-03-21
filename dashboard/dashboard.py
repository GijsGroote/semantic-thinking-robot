from dash import Dash, html, dcc
from dash.dependencies import Input, Output
import multiprocessing
import pandas as pd

pd.options.plotting.backend = "plotly"
from plotly.subplots import make_subplots
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
import pyarrow.feather as feather
from numpy import dstack


class Dashboard:

    def __init__(self):
        self.app = Dash(__name__)
        self.no_data_found = {
            "layout": {
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
            html.H1(children='Hello Dash'),

            html.Div(children='''
                Dash: A web application framework for your data.
            '''),
            html.Div([
                html.H4('MPC Live Feed'),
                html.Div(id='live-update-text'),
                dcc.Graph(id='live-update-mpc-graph', animate=True),
                dcc.Interval(
                    id='interval-component',
                    interval=1 * 1000,  # in milliseconds
                    n_intervals=0
                )
            ]),
        ])

        @self.app.callback(Output('live-update-text', 'children'),
                           Input('interval-component', 'n_intervals'))
        def update_metrics(n):
            return [
                html.Span('n: {0:.2f}'.format(n))
            ]

        @self.app.callback(Output('live-update-mpc-graph', 'figure'),
                           Input('interval-component', 'n_intervals'))
        def update_graph_live(n):

            # read in controller data
            df = feather.read_feather("../dashboard/mpc_data.feather")

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

                # fig['layout']['yaxis'].update(autorange=True)

                max_bound = max(15, df["time"][df.index[-1]]+1)
                fig.update_xaxes(range=[df["time"][0], max_bound])

                fig.update_yaxes(range=[dstack((df["x"], df["y"], df["theta"])).max()+0.2, dstack((df["x"], df["y"], df["theta"])).min()-0.2])

                # fig.update_yaxes(range=[dstack((df["u1"], df["u2"])).max() + 0.2,
                #                         dstack((df["u1"], df["u2"])).min() - 0.2])

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
