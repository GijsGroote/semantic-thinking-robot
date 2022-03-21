from dash import Dash, html, dcc
from dash.dependencies import Input, Output
import plotly
import plotly.express as px
import pandas as pd
import multiprocessing
import pandas as pd



class Dashboard:

    def __init__(self):
        self.app = Dash(__name__)

        self.app.layout = html.Div(children=[
            html.H1(children='Hello Dash'),

            html.Div(children='''
                Dash: A web application framework for your data.
            '''),

            html.Div([
                html.H4('MPC Live Feed'),
                dcc.Graph(id='live-update-mpc-graph', animate=True),
                dcc.Interval(
                    id='interval-component',
                    interval=1 * 1000,  # in milliseconds
                    n_intervals=0
                )
            ]),
            html.Div([
                html.H4('Live Feed test'),
                html.Div(id='live-update-text'),
                dcc.Graph(id='live-update-graph'),
                dcc.Interval(
                    id='interval-component',
                    interval=1 * 1000,  # in milliseconds
                    n_intervals=0
                )
            ])
        ])


        @self.app.callback(Output('live-update-mpc-graph', 'figure'),
                           Input('interval-component', 'n_intervals'))
        def update_mpc_graph_live(n):
            # todo: read data from disk an plot

            df = pd.read_csv('../dashboard/data.csv')
            return df.plot()





        @self.app.callback(Output('live-update-text', 'children'),
                           Input('interval-component', 'n_intervals'))
        def update_metrics(n):
            return [
                html.Span('Longitude: {0:.2f}'.format(n))
            ]

        # Multiple components can update everytime interval gets fired.
        @self.app.callback(Output('live-update-graph', 'figure'),
                           Input('interval-component', 'n_intervals'))
        def update_graph_live(n):
            df = pd.read_csv('../dashboard/data.csv')
            return df.plot()

    def startDashServer(self):
        def run():
            self.app.scripts.config.serve_locally = True
            self.app.run_server(
                port=8051,
                debug=False,
                processes=4,
                threaded=False
            )

        # Run on a separate process so that it doesn't block
        self.app.server_process = multiprocessing.Process(target=run)
        self.app.server_process.start()
