import os
import glob
import multiprocessing
from dash import Dash, html, dcc
from dashboard.callback import register_callbacks
from dashboard.figures import no_data_found_dict
# from IPython.display import display, HTML, Image
from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH

class Dashboard:
    """
    Dashboard class creates and updates a local site.
    """
    def __init__(self, app):

        # remove all old files
        files = glob.glob(PROJECT_PATH+"dashboard/data/*")
        for f in files:
            os.remove(f)

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
                border: 16px solid #f3f3f3; /* Light grey */
                border-top: 16px solid #3498db; /* Blue */
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
                100% {
                    transform: rotate(360deg);
                }
            }
        </style>
        </body>
        </html>
        """

        self.app.layout = html.Div(children=[
            dcc.Interval(
                id="interval-input",
                interval=1 * 1000,
                n_intervals=0
            ),
            html.Div([
                html.Div([
                    html.H4("Hypothesis Graph"),
                    html.Iframe(
                        id="hGraph",
                        srcDoc=self.loading_html,
                        className="graph"
                    ),
                    dcc.Interval(
                        id="hgraph-interval-component",
                        interval=1 * 1000,
                        n_intervals=0
                    )
                ], className="item"),
                html.Div([
                    html.H4("Controller Live Feed"),
                    dcc.Graph(id="live-update-controller-graph", animate=True),
                    dcc.Interval(
                        id="controller-interval-component",
                        interval=1 * 1000,  # in milliseconds
                        n_intervals=0
                        )
                    ], className="item"),
                html.Div([
                    html.H4("Knowledge Graph"),
                    html.Iframe(
                        id="kGraph",
                        srcDoc=self.loading_html,
                        className="graph"
                        ),
                    dcc.Interval(
                        id="kgraph-interval-component",
                        interval=1 * 1000,  # in milliseconds
                        n_intervals=0
                        )
                    ], className="item"),
                html.Div([
                    html.H4("Configuration Map"),
                    dcc.Graph(id="live-update-configuration-map"),
                    dcc.Interval(
                        id="configuration-map-interval-component",
                        interval=1 * 1000,  # in milliseconds
                        n_intervals=0
                        )
                    ], className="item"),
                html.Div([
                    html.H4("open space "),
                    html.Iframe(
                        srcDoc=self.loading_html,
                        className="graph"
                        ),
                    ], className="item"),

            ], className="container")
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
        app.scripts.config.serve_locally = True
        app.run_server(
            port=8052,
            debug=False,
            processes=4,
            threaded=False
        )

    # Run on a separate process so that it doesn"t block
    app.server_process = multiprocessing.Process(target=run)
    app.server_process.start()
