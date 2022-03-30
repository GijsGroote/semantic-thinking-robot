from dash.dependencies import Input, Output
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
from robot_brain.dashboard.figures import *


def register_callbacks(app):

    no_data_found_dict = create_no_data_found_dict(app)

    no_data_found_html = create_no_data_found_html(app)

    @app.callback(
        Output("hGraph", "srcDoc"), Input('controller-interval-component', 'n_intervals'), prevent_initial_call=True)
    def update_hgraph_live(input_value):

        net = Network(bgcolor=app.figure_background_color, height="450px")

        hgraph = HGraph()


        hgraph.addTargetNode(ConfSetNode(2, "P", []))
        hgraph.addNode(ConfSetNode(3, "P", []))
        hgraph.addNode(ConfSetNode(4, "P", []))
        hgraph.addNode(ConfSetNode(1, "P", []))
        hgraph.addNode(ConfSetNode(5, "P", []))
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
            net.add_node(node.id, label="Node(1, confSet, the one thing\n the other thing\n another thing\ another thing)")

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
            return no_data_found_dict

        else:
            df = feather.read_feather("../robot_brain/dashboard/data/mpc_data.feather")
            # todo: this can be done better, send metadata with the dataframe
            if df.type[0] == "mpc":
                return create_mpc_plot(app, df)


