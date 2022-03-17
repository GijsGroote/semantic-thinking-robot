from dash import Dash, html, dcc
import plotly.express as px
import pandas as pd
import multiprocessing


app = Dash(__name__)

# assume you have a "long-form" data frame
# see https://plotly.com/python/px-arguments/ for more options
df = pd.DataFrame({
    "Fruit": ["Apples", "Oranges", "Bananas", "Apples", "Oranges", "Bananas"],
    "Amount": [4, 1, 2, 2, 4, 5],
    "City": ["SF", "SF", "SF", "Montreal", "Montreal", "Montreal"]
})

fig = px.bar(df, x="Fruit", y="Amount", color="City", barmode="group")

app.layout = html.Div(children=[
    html.H1(children='Hello Dash'),

    html.Div(children='''
        Dash: A web application framework for your data.
    '''),

    dcc.Graph(
        id='example-graph',
        figure=fig
    )
])


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





