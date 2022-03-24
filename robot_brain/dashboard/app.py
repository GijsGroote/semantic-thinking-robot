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

net = Network()
hgraph = HGraph()
hgraph.addTargetNode(ConfSetNode(2, "P", []))
hgraph.addNode(ConfSetNode(3, "P", []))
hgraph.addNode(ConfSetNode(7, "P", []))
hgraph.addNode(ConfSetNode(1, "P", []))
hgraph.addNode(ObjectSetNode(5, "P", []))
hgraph.addEdge(Edge("id", 2, 3, "verb", "controller"))
hgraph.addEdge(Edge("id", 7, 1, "verb", "controller"))
hgraph.addEdge(Edge("id", 3, 3, "verb", "controller"))
hgraph.addEdge(Edge("id", 7, 5, "verb", "controller"))
# add nodes
for node in hgraph.nodes:
    net.add_node(node.id, label=node.id)

# add edges
for edge in hgraph.edges:
    net.add_edge(edge.source, edge.to)

# net.show("name.html")

class Dashboard:

    def __init__(self):
        self.app = Dash(__name__, title="DashBoard", update_title=None)
        self.no_data_found = {
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


        self.app.layout = html.Div(children=[
            html.Div([
                html.Div([
                    html.H4('HGraph'),
                    html.Iframe(srcDoc=net.html, className="graph")
                    # dcc.Graph(id='live-update-hgraph', animate=False),
                    # dcc.Interval(
                    #     id='hgraph-interval-component',
                    #     interval=1 * 1000,  # in milliseconds
                    #     n_intervals=0
                    # )
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
                    dcc.Graph(figure=self.no_data_found)
                ], className="item"),
                html.Div([
                    html.H4('extra plot space'),
                    dcc.Graph(figure=self.no_data_found)
                ], className="item"),

            ], className="container")
        ])

        register_callbacks(self.app)

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
