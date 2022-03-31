from dash import Dash, html, dcc
import multiprocessing
import os
from robot_brain.dashboard.callback import register_callbacks
from IPython.display import display, HTML, Image
from robot_brain.global_variables import *


class Dashboard:

    def __init__(self, app):
        app.controller_graph_ready = False
        # todo: other graphs
        self.app = app

        # todo: no data found should be in the callback, only loading screen please
        no_data_found_dict = {
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
                background-color: ''' + FIG_BG_COLOR + ''';
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
                background-color: ''' + FIG_BG_COLOR+ ''';
        
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

        # THIS LOADING IFRAME COULD BE USEFULL
        # html.Iframe(
        #     srcDoc=self.loading_html,
        #     className="graph"
        # )

        self.app.layout = html.Div(children=[
            dcc.Interval(
                id='interval-input',
                interval=1 * 1000,
                n_intervals=0
            ),
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
                        interval=1 * 1000,
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

    # create dashboard
    Dashboard(app)

    def run():
        app.scripts.config.serve_locally = True
        app.run_server(
            port=8051,
            debug=False,
            processes=4,
            threaded=False
        )

    # Run on a separate process so that it doesn't block
    app.server_process = multiprocessing.Process(target=run)
    app.server_process.start()
