import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import numpy as np
from numpy import dstack

pd.options.plotting.backend = "plotly"

from robot_brain.global_variables import FIG_BG_COLOR, CREATE_SERVER_DASHBOARD
from robot_brain.graph.conf_set_node import ConfSetNode
from robot_brain.graph.object_set_node import ObjectSetNode
from robot_brain.graph.h_graph import HGraph

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
    # honestly this could also be done from the RBrain.py every so what seconds...
    graph.visualise(path)

    
def create_mpc_plot(df):

        fig = make_subplots(rows=2, cols=1)
        
        time = df["time"]

        # x, y and theta positions
        fig.append_trace(go.Scatter(
            x=time,
            y=df["x"],
            name="x-position",
            line=dict(color='medium purple')
            ), row=1, col=1)
        fig.append_trace(go.Scatter(
            x=time,
            y=df["y"],
            name="y-position",
            line=dict(color='forest  green')
            ), row=1, col=1)
        fig.append_trace(go.Scatter(
            x=time,
            y=df["theta"],
            name="orientation",
            line=dict(color='dark green')
            ), row=1, col=1)

        # reference signals
        fig.append_trace(go.Scatter(
            x=[time[0], time.index[-1]],
            y=df["x_ref"][0]*np.ones((2,)),
            name="x-ref",
            line=dict(color='medium purple', width=1, dash='dash')
            ), row=1, col=1)
        fig.append_trace(go.Scatter(
            x=[time[0], time.index[-1]],
            y=df["y_ref"][0]*np.ones((2,)),
            name="y-ref",
            line=dict(color='forest green', width=1, dash='dash')
            ), row=1, col=1)
        fig.append_trace(go.Scatter(
            x=[time[0], time.index[-1]],
            y=df["theta_ref"][0]*np.ones((2,)),
            name="orientation-ref",
            line=dict(color='dark green', width=1, dash='dash')
            ), row=1, col=1)

        # input
        fig.append_trace(go.Scatter(
            x=time,
            y=df["u1"],
            name="u1",
            line=dict(color='silver', shape='hv')
        ), row=2, col=1)
        fig.append_trace(go.Scatter(
            x=time,
            y=df["u2"],
            name="u2",
            line=dict(color='gray', shape='hv'),
        ), row=2, col=1)

        # prediction error plot
        fig.append_trace(go.Scatter(
            x=time,
            y=df["pred_error"],
            name="prediction error",
            line=dict(color='red'),
        ), row=2, col=1)        

        # scale the axis
        fig.update_xaxes(range=[df["time"][0], max(15, df["time"][df.index[-1]] + 1)],
                         row=1, col=1)

        fig.update_xaxes(range=[df["time"][0], max(15, df["time"][df.index[-1]] + 1)],
                         title_text="Time [sec]",
                         row=2, col=1)

        fig.update_yaxes(range=[dstack((df["x"], df["y"], df["theta"])).min() - 0.2,
                                dstack((df["x"], df["y"], df["theta"])).max() + 0.2],
                         title_text="position [-]",
                         row=1, col=1)

        fig.update_yaxes(range=[dstack((df["u1"], df["u2"])).max() + 0.2,
            dstack((df["u1"], df["u2"])).min() - 0.2],
            title_text="input [-] & error [-]",
            row=2, col=1)

        fig.update_layout({"title": {"text": "MPC controller"}})

        fig.update_layout(paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

        return fig
