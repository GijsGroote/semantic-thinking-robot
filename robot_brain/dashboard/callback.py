import os
import time
from pathlib import Path
from pyarrow import feather
from dash.dependencies import Input, Output
from dash.exceptions import PreventUpdate
import pandas as pd
pd.options.plotting.backend = "plotly"
from robot_brain.dashboard.figures import *


def check_file_is_up_to_date(path):
    # prevent updating if file is older than 3 seconds

    if time.time() - os.path.getmtime(path) > 3:
        raise PreventUpdate # will show 304 status code, which is ok


def register_callbacks(app):

    no_data_found_dict = create_no_data_found_dict(app)
    no_data_found_html = create_no_data_found_html(app)

    @app.callback(
        Output("hGraph", "srcDoc"), Input('controller-interval-component', 'n_intervals'))
    def update_hgraph_live(n):

        path = "../robot_brain/dashboard/data/hgraph.html"

        # read in controller data if it exists
        if not Path(path).is_file():
            return create_no_data_found_html(app)
        else:
            # only update up-to-date files, exception for n = 0
            if n > 0:
                check_file_is_up_to_date(path)

            # open text file in read mode
            with open(path, "r", encoding="urf-8") as file:
                data = file.read()

            return data

    @app.callback(
        Output("kGraph", "srcDoc"), Input('controller-interval-component', 'n_intervals'))
    def update_kgraph_live(n):

        path = "../robot_brain/dashboard/data/kgraph.html"

        # read in controller data if it exists
        if not Path(path).is_file():
            return create_no_data_found_html(app)
        else:
            # only update up-to-date files, exception for n = 0
            if n > 0:
                check_file_is_up_to_date(path)

            # open text file in read mode
            with open(path, "r", encoding="urf-8") as file:
                data = file.read()

            return data


    @app.callback(Output('live-update-controller-graph', 'figure'),
                       Input('controller-interval-component', 'n_intervals'))
    def update_controller_graph_live(n):

        # read in controller data if it exists
        if not Path("../robot_brain/dashboard/data/mpc_data.feather").is_file():
            return no_data_found_dict

        else:
            # only update up-to-date files, exception for n = 0
            if n > 0:
                check_file_is_up_to_date("../robot_brain/dashboard/data/mpc_data.feather")

            df = feather.read_feather("../robot_brain/dashboard/data/mpc_data.feather")

            # todo: this can be done better, send metadata with the dataframe

            if df.type[0] == "mpc":
                return create_mpc_plot(df)
