import pandas as pd


from robot_brain.global_variables import *

from robot_brain.graphs.ConfSetNode import ConfSetNode

from robot_brain.graphs.ObjectSetNode import ObjectSetNode
from robot_brain.graphs.HGraph import HGraph
from pyvis.network import Network

pd.options.plotting.backend = "plotly"
import os
from plotly.subplots import make_subplots
import plotly.graph_objects as go
from numpy import dstack


def create_no_data_found_dict(app):
    return {
        "layout": {
            "paper_bgcolor": FIG_BG_COLOR,
            "plot_bgcolor": FIG_BG_COLOR,
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
                        "family": "Ubuntu Mono",
                    }
                }
            ]
        }
    }


def create_no_data_found_html(app):
    return '''
            <!DOCTYPE html>
            <html lang="en">
            <head>
                <meta charset="UTF-8">
                <title>No Data found</title>
            </head>
            <body>
            <div id="no_data_found_div">
            <p> No Graph Data Found </p>
            </div>
    
            <style type="text/css">
            body {
                margin: 0px;
            }
    
            #no_data_found_div {
                width: 100%;
                height: 0px;
                text-align: center;
                padding: 200px 0 250px 0;
                font-size: 25px;
                font-family: Ubuntu Mono;
                background-color: ''' + FIG_BG_COLOR + ''';
            }
    
            </style>
            </body>
            </html>
            '''

def create_graph_plot(graph, path):
    print("graph has been written to disk")
    net = Network(bgcolor=FIG_BG_COLOR, height="450px")

    # add nodes
    for node in graph.nodes:
        net.add_node(node.id, label="Here you can plot stuff")

    # add edges
    for edge in graph.edges:
        net.add_edge(edge.source, edge.to)

    # set a custom style sheet
    net.path = os.getcwd() + "/../robot_brain/dashboard/assets/graph_template.html"

    net.write_html(path)



def create_mpc_plot(df):
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
    fig.update_layout(paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

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
