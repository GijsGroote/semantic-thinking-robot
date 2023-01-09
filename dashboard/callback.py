import os
import time
import pickle
from pathlib import Path
from dash.dependencies import Input, Output
from dash.exceptions import PreventUpdate
import pandas as pd
pd.options.plotting.backend = "plotly"

from dashboard.figures import create_no_data_found_dict, create_no_data_found_html
from robot_brain.global_variables import PROJECT_PATH

from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH

def check_file_is_up_to_date(path):
    # prevent updating if file is older than 3 seconds
    if time.time() - os.path.getmtime(path) > 3:
        raise PreventUpdate # will show 304 status code, which is ok

def register_callbacks(app):

    no_data_found_dict = create_no_data_found_dict(app)

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

    @app.callback(Output("controller", "figure"),
                       Input("controller-interval-component", "n_intervals"))
    def update_controller_graph_live(n):

        file_path = PROJECT_PATH+"dashboard/data/cntrl.pickle"

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

    @app.callback(Output("pe", "figure"),
            Input("pe-interval-component", "n_intervals"))

    def update_path_estimator(n):

        file_path = PROJECT_PATH+"dashboard/data/cgrid.pickle"
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
                        autosize=True,
                        height=500,
                        margin=dict(
                            l=50,
                            r=50,
                            b=50,
                            t=50,
                            pad=4
                            ),
                        paper_bgcolor=FIG_BG_COLOR,
                        plot_bgcolor=FIG_BG_COLOR)

                return fig

    @app.callback(Output("mp", "figure"),
            Input("mp-interval-component", "n_intervals"))

    def update_motion_planner(n):

        file_path = PROJECT_PATH+"dashboard/data/mp.pickle"
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
                        autosize=True,
                        height=500,
                        margin=dict(
                            l=50,
                            r=50,
                            b=50,
                            t=50,
                            pad=4
                            ),
                        paper_bgcolor=FIG_BG_COLOR,
                        plot_bgcolor=FIG_BG_COLOR)
            return fig

    close_popup = {
        'display': 'block',
        'border': 'none',
        'width': '100%',
        'height': '450',
    }

    open_graph_popup = {'display': 'block',
                'margin': 'auto',
                'position': 'fixed',
                'z-index': '2',
                'left': '7%',
                'top': '15%',
                'width': '86%',
                '-webkit-animation-name': 'zoom',
                '-webkit-animation-duration': '0.6s',
                'animation-name': 'zoom',
                'animation-duration': '0.6s',
                }

    open_plotly_popup = {'display': 'block',
                'margin': 'auto',
                'position': 'fixed',
                'z-index': '2',
                'left': '12%',
                'top': '15%',
                'width': '76%',
                '-webkit-animation-name': 'zoom',
                '-webkit-animation-duration': '0.6s',
                'animation-name': 'zoom',
                'animation-duration': '0.6s',
                }

    @app.callback(Output('modal_background', 'style'),
            Input('hgraph_title', 'n_clicks_timestamp'),
            Input('kgraph_title', 'n_clicks_timestamp'),
            Input('pe_title', 'n_clicks_timestamp'),
            Input('mp_title', 'n_clicks_timestamp'),
            Input('controller_title', 'n_clicks_timestamp'),
            Input('modal_background', 'n_clicks_timestamp'),
            prevent_initial_call=True)
    def open_modal_background(a,b,c,d,e,m_b):

        if (int(m_b) > int(a) and
            int(m_b) > int(b) and
            int(m_b) > int(c) and
            int(m_b) > int(d) and
            int(m_b) > int(e)):
            return {'display': 'none'}
        else:
            return {'display': 'block',
                'z-index': '1',
                'left': '0',
                'top': '0',
                'width': '100%',
                'height': '100%',
                'overflow': 'auto',
                'position': 'fixed',
                'background-color': 'rgba(0,0,0,0.9)',
                }

    @app.callback(Output('hGraph', 'style'),
        [Input('hgraph_title', 'n_clicks_timestamp'),
        Input('modal_background', 'n_clicks_timestamp')],
        prevent_initial_call=True)
    def hgraph_popup(a, b):
        if int(a) > int(b):
            return open_graph_popup
        else:
            return close_popup

    @app.callback(Output('kGraph', 'style'),
        [Input('kgraph_title', 'n_clicks_timestamp'),
        Input('modal_background', 'n_clicks_timestamp')],
        prevent_initial_call=True)
    def kgraph_popup(a, b):

        if int(a) > int(b):
            return open_graph_popup
        else:
            return close_popup

    @app.callback(Output('pe', 'style'),
        [Input('pe_title', 'n_clicks_timestamp'),
        Input('modal_background', 'n_clicks_timestamp')],
        prevent_initial_call=True)
    def pe_popup(a, b):

        if int(a) > int(b):
            return open_plotly_popup
        else:
            return close_popup

    @app.callback(Output('mp', 'style'),
        [Input('mp_title', 'n_clicks_timestamp'),
        Input('modal_background', 'n_clicks_timestamp')],
        prevent_initial_call=True)
    def mp_popup(a, b):

        if int(a) > int(b):
            return open_plotly_popup
        else:
            return close_popup

    @app.callback(Output('controller', 'style'),
        [Input('controller_title', 'n_clicks_timestamp'),
        Input('modal_background', 'n_clicks_timestamp')],
        prevent_initial_call=True)

    def controller_popup(a, b):
        if int(a) > int(b):
            return open_plotly_popup
        else:
            return close_popup
