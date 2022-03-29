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

    graph_background_color = "rgba(229,236,246,255)"

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
                    "background-color": graph_background_color,
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



    @app.callback(
        Output("my-output", "srcDoc"), Input('controller-interval-component', 'n_intervals'), prevent_initial_call=True
    )
    def update_hgraph_live(input_value):
        import random

        # add 22px to the height because it has negative 22 margin-top
        net = Network(bgcolor=graph_background_color, height="472px")

        hgraph = HGraph()
        # rand = random.randint(2, 5)
        #

        # rand2 = random.randint(1, 4)
        # print("rand1: {}, rand2: {}".format(rand, rand2))
        hgraph.addTargetNode(ConfSetNode(2, "P", []))
        hgraph.addNode(ConfSetNode(3, "P", []))
        hgraph.addNode(ConfSetNode(4, "P", []))
        hgraph.addNode(ConfSetNode(1, "P", []))
        if input_value > 2:
            hgraph.addNode(ObjectSetNode(5, "P", []))
        if input_value > 3:
            hgraph.addEdge(Edge("id", 2, 5, "verb", "controller"))
        if input_value > 5:
            hgraph.addNode(ObjectSetNode(6, "P", []))
        if input_value > 6:
            hgraph.addEdge(Edge("id", 2, 6, "verb", "controller"))

        hgraph.addEdge(Edge("id", 2, 3, "verb", "controller"))
        hgraph.addEdge(Edge("id", 4, 5, "verb", "controller"))
        # add nodes
        for node in hgraph.nodes:
            net.add_node(node.id, label=node.id)

        # add edges
        for edge in hgraph.edges:
            net.add_edge(edge.source, edge.to)

        # set a custom style sheet
        net.path = os.getcwd()+"/../robot_brain/dashboard/assets/graph_template.html"
        net.write_html("../robot_brain/dashboard/data/hgraph_test.html")


        return net.html



    @app.callback(Output('live-update-controller-graph', 'figure'),
                       Input('controller-interval-component', 'n_intervals'))
    def update_controller_graph_live(n):
        # read in controller data if it exists
        if not Path("../robot_brain/dashboard/data/mpc_data.feather").is_file():
            return no_data_found

        else:
            df = feather.read_feather("../robot_brain/dashboard/data/mpc_data.feather")

            # todo: this can be done better, send metadata with the dataframe
            if df.type[0] == "mpc":
                fig = make_subplots(rows=2, cols=1)
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
