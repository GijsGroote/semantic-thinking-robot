import os
import time
import pickle
from pathlib import Path
from robot_brain.global_variables import PROJECT_PATH
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
        Output("hGraph", "srcDoc"), Input("hgraph-interval-component", "n_intervals"))
    def update_hgraph_live(n):


        path = PROJECT_PATH+"dashboard/data/hypothesis_graph.html"

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
        Output("kGraph", "srcDoc"), Input("kgraph-interval-component", "n_intervals"))
    def update_kgraph_live(n):

        path = PROJECT_PATH+"dashboard/data/knowledge_graph.html"

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

        file_path = PROJECT_PATH+"dashboard/data/controller.pickle"
        
        # read in controller data if it exists
        if not Path(file_path).is_file():
            return no_data_found_dict

        else:
            # only update up-to-date files, exception for n = 0
            if n > 0:
                check_file_is_up_to_date(file_path)

            with open(file_path, "rb") as file:

                fig = pickle.load(file)
                                       
                return fig

    @app.callback(Output("live-update-configuration-map", "figure"),
            Input("configuration-map-interval-component", "n_intervals"))

    def update_configuration_map(n):

        file_path = PROJECT_PATH+"dashboard/data/configuration_grid.pickle"
        # read in controller data if it exists
        if not Path(file_path).is_file():
            return no_data_found_dict

        else:
            # only update up-to-date files, exception for n = 0
            if n > 0:
                check_file_is_up_to_date(file_path)

            with open(file_path, "rb") as file:

                fig = pickle.load(file)
                fig.update_layout(
                        paper_bgcolor=FIG_BG_COLOR,
                        plot_bgcolor=FIG_BG_COLOR)
                        
                return fig

    @app.callback(Output("live-update-motion-planner", "figure"),
            Input("motion-planner-interval-component", "n_intervals"))

    def update_motion_planner(n):

        file_path = PROJECT_PATH+"dashboard/data/motion_planner.pickle"
        # read in controller data if it exists
        if not Path(file_path).is_file():
            return no_data_found_dict

        else:
            # only update up-to-date files, exception for n = 0
            if n > 0:
                check_file_is_up_to_date(file_path)

            with open(file_path, "rb") as file:

                fig = pickle.load(file)
                        
            return fig

