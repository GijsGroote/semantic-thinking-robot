from dash import Dash, html, dcc
from dash.dependencies import Input, Output
import plotly
import plotly.express as px
import pandas as pd
import multiprocessing


class Dashboard:

    def __init__(self):
        self.app = Dash(__name__)
        self.controller_fig = None
        self.hgraph_fig = None

        # assume you have a "long-form" data frame
        # see https://plotly.com/python/px-arguments/ for more options
        df = pd.DataFrame({
            "Fruit": ["Apples", "Oranges", "Bananas", "Apples", "Oranges", "Bananas"],
            "Amount": [4, 1, 2, 2, 4, 5],
            "City": ["SF", "SF", "SF", "Montreal", "Montreal", "Montreal"]
        })

        fig = px.bar(df, x="Fruit", y="Amount", color="City", barmode="group")



        self.app.layout = html.Div(children=[
            html.H1(children='Hello Dash'),

            html.Div(children='''
                Dash: A web application framework for your data.
            '''),

            dcc.Graph(
                id='example-graph',
                figure=fig
            ),
            html.Div([
                html.H4('MPC Live Feed'),
                html.Div(id='live-update-text'),
                dcc.Graph(id='live-update-graph'),
                dcc.Interval(
                    id='interval-component',
                    interval=1 * 1000,  # in milliseconds
                    n_intervals=0
                )
            ])
        ])


        @app.callback(Output('live-update-text', 'children'),
                      Input('interval-component', 'n_intervals'))


        def update_metrics(n):
            return [
                html.Span('Longitude: {0:.2f}'.format(n))
                ]


        # Multiple components can update everytime interval gets fired.
        @app.callback(Output('live-update-graph', 'figure'),
                      Input('interval-component', 'n_intervals'))

        def update_controller_graph(n):
            fig = px.scatter(x=[0, 1, 2, 3, 4], y=[0, 1, 4, 9, 16])
            return fig





    def startDashServer(app):
        def run():
            app.scripts.config.serve_locally = True
            app.run_server(
                port=8050,
                debug=False,
                processes=4,
                threaded=False
            )
        # Run on a separate process so that it doesn't block
        app.server_process = multiprocessing.Process(target=run)
        app.server_process.start()





