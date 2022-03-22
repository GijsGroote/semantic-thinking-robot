from dash import Dash, html, dcc
from dash.dependencies import Input, Output
import multiprocessing
import pandas as pd
from os.path import exists
import dash_dangerously_set_inner_html

pd.options.plotting.backend = "plotly"
from plotly.subplots import make_subplots
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
from pathlib import Path
import pyarrow.feather as feather
from numpy import dstack

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

net.show("name.html")

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
