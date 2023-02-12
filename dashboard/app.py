import os
import glob
import multiprocessing
from dash import Dash, html, dcc
from dashboard.callback import register_callbacks

from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH, DASHBOARD_PORT_PID


class Dashboard:
    """
    Dashboard class creates and updates a local site.
    """
    def __init__(self, app):

        # remove all old files
        files = glob.glob(PROJECT_PATH+"dashboard/data/*")
        for file in files:
            os.remove(file)

        self.app = app

        self.loading_html = """
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
                background-color: """ + FIG_BG_COLOR + """;
                height: 450px;
            }
        
            #no_data_found_div {
                position: absolute;
                top: calc(50% - 33px);
                left: calc(50% - 33px);
                transform: translate(-50%, -50%);
                border: */
                border-top: */
                border-radius: 50%;
                width: 50px;
                height: 50px;
                animation: spin 2s linear infinite;
                margin: auto;
                background-color: """ + FIG_BG_COLOR+ """;
        
            }
        
            @keyframes spin {
                0% {
                    transform: rotate(0deg);
                }
                {
                    transform: rotate(360deg);
                }
            }
        </style>
        </body>
        </html>
        """
        self.app.layout = html.Div(children=[

            # html.Article(dji.Import(src=PROJECT_PATH+"popup.js")),
            html.Div(id="modal_background", n_clicks_timestamp='0'),

            # dcc.Interval(
            #     id="interval-input",
            #     interval=1000,
            #     n_intervals=0
            # ),
            html.Div([
                html.Div([
                    html.H4("Hypothesis Graph", className="item_title", id="hgraph_title", n_clicks_timestamp='0'),
                    html.Iframe(
                        id="hGraph",
                        srcDoc=self.loading_html,
                        className="graph"
                    ),
                    dcc.Interval(
                        id="hgraph-interval-component",
                        interval=1000,
                        n_intervals=0
                    )
                ], className="item", id="hgraph_div"),

                html.Div([
                    html.H4("Knowledge Graph", className="item_title", id="kgraph_title", n_clicks_timestamp='0'),
                    html.Iframe(
                        id="kGraph",
                        srcDoc=self.loading_html,
                        className="graph"
                        ),
                    dcc.Interval(
                        id="kgraph-interval-component",
                        interval=1000,
                        n_intervals=0
                        )
                    ], className="item"),

                html.Div([
                    html.H4("Path Existence Estimator", className="item_title", id="pe_title", n_clicks_timestamp='0'),
                    dcc.Graph(id="pe"),
                    dcc.Interval(
                        id="pe-interval-component",
                        interval=1000,
                        n_intervals=0
                        )
                    ], className="item"),

                html.Div([
                    html.H4("Motion Planner", className="item_title", id="mp_title", n_clicks_timestamp='0'),
                    dcc.Graph(id="mp"),
                    dcc.Interval(
                        id="mp-interval-component",
                        interval=1000,
                        n_intervals=0
                        )
                    ], className="item"),


                html.Div([
                    html.H4("Controller Live Feed", className="item_title", id="controller_title", n_clicks_timestamp='0'),
                    dcc.Graph(id="controller", animate=True),
                    dcc.Interval(
                        id="controller-interval-component",
                        interval=1000,
                        n_intervals=0
                        )
                    ], className="item"),

                html.Div([
                    html.H4("open spacsssse "),
                    html.Iframe(
                        srcDoc=self.loading_html,
                        className="graph"
                        ),
                    ], className="item"),

            ], className="container"),
        ])
        register_callbacks(self.app)


def start_dash_server():
    # change working directory
    os.chdir(PROJECT_PATH+"environments/")

    # create app
    app = Dash(__name__,
            title="DashBoard",
            update_title=None,
            assets_folder="assets")

    # create dashboard
    Dashboard(app)

    def run():
        app.scripts.config.serve_locally = False
        app.run_server(
            port=DASHBOARD_PORT_PID,
            debug=False,
            processes=4,
            threaded=False
        )

    # Run on a separate process so that it doesn"t block
    app.server_process = multiprocessing.Process(target=run)
    app.server_process.start()
    return app.server_process 

def stop_dash_server(dash_app):
    """ kills the process, thereby terminating the dash server. """
    dash_app.kill()

