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

class Dashboard:

    def __init__(self, app):
        self.app = app
        self.no_data_found_fig = {
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
        self.no_data_found_html = '''
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <meta charset="UTF-8">
            <title>No Data found</title>
        </head>
        <body>
        <div id="no_data_found_div">
        No data found
        </div>
        
        <style type="text/css">
        
        #no_data_found_div {
            width: 500px;
            height: 600px;
            background-color: orange;
            border: 1px solid lightgray;
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
                        id="my-output",
                        srcDoc=self.no_data_found_html,
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
                    dcc.Graph(figure=self.no_data_found_fig)
                ], className="item"),
                html.Div([
                    html.H4('extra plot space'),
                    dcc.Graph(figure=self.no_data_found_fig)
                ], className="item"),

            ], className="container")
        ])

        register_callbacks(self.app)

def startDashServer():

    # change working directory
    os.chdir("/home/gijs/Documents/semantic-thinking-robot/environments/")

    # create app
    app = Dash(__name__, title="DashBoard", update_title=None)

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
