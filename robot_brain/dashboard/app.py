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
from robot_brain.dashboard.callback import register_callbacks
from robot_brain.graphs.Edge import Edge

from robot_brain.graphs.ConfSetNode import ConfSetNode

from robot_brain.graphs.ObjectSetNode import ObjectSetNode
from robot_brain.graphs.HGraph import HGraph
from pyvis.network import Network
import time


class Dashboard:

    def __init__(self, app):
        self.app = app

        no_data_found_dict = {
            "layout": {
                "paper_bgcolor": app.figure_background_color,
                "plot_bgcolor": app.figure_background_color,
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
                            "size": 25,
                            "color": "black",
                        }
                    }
                ]
            }
        }

        self.loading_html = '''
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <meta charset="UTF-8">
            <title> Loading </title>
        </head>
        <body>
        <div id="no_data_found_div">
        
        </div>
        
        <style type="text/css">
            body {
                display: block;
                margin: 0px;
                background-color: ''' + app.figure_background_color + ''';
                height: 450px;
            }
        
            #no_data_found_div {
                position: absolute;
                top: calc(50% - 33px);
                left: calc(50% - 33px);
                transform: translate(-50%, -50%);
                border: 16px solid #f3f3f3; /* Light grey */
                border-top: 16px solid #3498db; /* Blue */
                border-radius: 50%;
                width: 50px;
                height: 50px;
                animation: spin 2s linear infinite;
                margin: auto;
                background-color: ''' + app.figure_background_color + ''';
        
            }
        
            @keyframes spin {
                0% {
                    transform: rotate(0deg);
                }
                100% {
                    transform: rotate(360deg);
                }
            }
        </style>
        </body>
        </html>
        '''

        self.app.layout = html.Div(children=[
            html.Div([
                html.Div([
                    html.H4('HGraph'),
                    html.Iframe(
                        id="hGraph",
                        srcDoc=self.loading_html,
                        className="graph"
                    ),
                    dcc.Interval(
                        id='hgraph-interval-component',
                        interval=1 * 1000,  # in milliseconds
                        n_intervals=0
                    )
                ], className="item"),
                html.Div([
                    html.H4('Controller Live Feed'),
                    dcc.Graph(id='live-update-controller-graph', animate=True),
                    dcc.Interval(
                        id='controller-interval-component',
                        interval=1 * 1000,  # in milliseconds
                        n_intervals=0
                    )
                ], className="item"),

                html.Div([
                    html.H4('KGraph, todo'),
                    html.Iframe(
                        id="kGraph",
                        srcDoc=self.loading_html,
                        className="graph"
                    ),
                    dcc.Interval(
                        id='kgraph-interval-component',
                        interval=1 * 1000,  # in milliseconds
                        n_intervals=0
                    )
                ], className="item"),
                html.Div([
                    html.H4('extra plot space'),
                    dcc.Graph(figure=no_data_found_dict),
                ], className="item"),

            ], className="container")
        ])

        register_callbacks(self.app)


def startDashServer():
    # change working directory
    os.chdir("/home/gijs/Documents/semantic-thinking-robot/environments/")

    # create app
    app = Dash(__name__, title="DashBoard", update_title=None)

    # color styles
    app.figure_background_color = "rgba(229,236,246,255)"

    # create dashboard
    Dashboard(app)

    def run():
        app.scripts.config.serve_locally = True
        app.run_server(
            port=8050,
            debug=False,
            processes=4,
            threaded=False
        )

    # Run on a separate process so that it doesn't block
    app.server_process = multiprocessing.Process(target=run)
    app.server_process.start()
