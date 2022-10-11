import os
import time
import pickle
from pathlib import Path
from pyarrow import feather
from dash.dependencies import Input, Output
from dash.exceptions import PreventUpdate
import pandas as pd
pd.options.plotting.backend = "plotly"
from dashboard.figures import *


def check_file_is_up_to_date(path):
    # prevent updating if file is older than 3 seconds

    if time.time() - os.path.getmtime(path) > 3:
        raise PreventUpdate # will show 304 status code, which is ok


def register_callbacks(app):

    no_data_found_dict = create_no_data_found_dict(app)
    no_data_found_html = create_no_data_found_html(app)

    @app.callback(
        Output("hGraph", "srcDoc"), Input("controller-interval-component", "n_intervals"))
    def update_hgraph_live(n):


        # TODO: do not go for a hardcoded line!
        path = "/home/gijs/Documents/semantic-thinking-robot/dashboard/data/hgraph.html"

        # read in controller data if it exists
        if not Path(path).is_file():
            return create_no_data_found_html(app)
        else:
            # only update up-to-date files, exception for n = 0
            if n > 0:
                check_file_is_up_to_date(path)

            # open text file in read mode
            with open(path, "r") as file:
                data = file.read()

            return data

    @app.callback(
        Output("kGraph", "srcDoc"), Input("controller-interval-component", "n_intervals"))
    def update_kgraph_live(n):

        path = "/home/gijs/Documents/semantic-thinking-robot/dashboard/data/kgraph.html"

        # read in controller data if it exists
        if not Path(path).is_file():
            return create_no_data_found_html(app)
        else:
            # only update up-to-date files, exception for n = 0
            if n > 0:
                check_file_is_up_to_date(path)

            # open text file in read mode
            with open(path, "r") as file:
                data = file.read()

            return data


    @app.callback(Output("live-update-controller-graph", "figure"),
                       Input("controller-interval-component", "n_intervals"))
    def update_controller_graph_live(n):

        # read in controller data if it exists
        if not Path("../dashboard/data/mpc_data.feather").is_file():
            return no_data_found_dict

        else:
            # only update up-to-date files, exception for n = 0
            if n > 0:
                check_file_is_up_to_date("../dashboard/data/mpc_data.feather")

            df = feather.read_feather("../dashboard/data/mpc_data.feather")

            # todo: this can be done better, send metadata with the dataframe

            if df.type[0] == "mpc":
                return create_mpc_plot(df)

    @app.callback(Output("live-update-occupancy-map", "figure"),
                       Input("occupancy-map-interval-component", "n_intervals"))

    def update_occupancy_map(n):
        # read in controller data if it exists
        if not Path("../dashboard/data/occupancy_map.pickle").is_file():
            return no_data_found_dict

        else:
            # only update up-to-date files, exception for n = 0
            if n > 0:
                check_file_is_up_to_date("../dashboard/data/occupancy_map.pickle")

            with open("../dashboard/data/occupancy_map.pickle", "rb") as file:

                fig = pickle.load(file)
                fig.update_layout(
                        paper_bgcolor=FIG_BG_COLOR,
                        plot_bgcolor=FIG_BG_COLOR)
                        
                return fig
