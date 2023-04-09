import glob

import json
from plotly.subplots import make_subplots
import plotly.graph_objects as go

from robot_brain.global_variables import PROJECT_PATH

def create_time_figure(data_path: str, save_path=False):

    total_time = []
    execute_time = []
    search_time = []
    subtask_number = []


    # collect date from json files
    json_file_paths = glob.glob(data_path)

    for (i, json_file_path) in enumerate(json_file_paths):
        subtask_number.append(i)
        # Opening JSON file
        with open(json_file_path) as json_file:
            data = json.load(json_file)
            total_time.append(data["total_time"])
            execute_time.append(data["execute_time"])
            search_time.append(data["search_time"])

    fig = make_subplots(rows=1, cols=1)
    fig.append_trace(go.Scatter(
        x=subtask_number,
        y=total_time,
        name="Total time",
        line=dict(color='medium purple', width=1, dash='dash')
        ), row=1, col=1)
    fig.append_trace(go.Scatter(
        x=subtask_number,
        y=execute_time,
        name="Execute time",
        line=dict(color='forest green', width=1, dash='dash')
        ), row=1, col=1)
    fig.append_trace(go.Scatter(
        x=subtask_number,
        y=search_time,
        name="Search Time",
        line=dict(color='forest green', width=1, dash='dash')
        ), row=1, col=1)

    fig.update_xaxes(title_text="Task number", row=1, col=1)
    fig.update_yaxes(title_text="Time [sec]", row=1, col=1)

    fig.update_layout({"title": {"text": "Search and Execution times"}})

    # fig.update_layout(paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

    if save_path is False:
        fig.show()
    else:
        fig.write_image(save_path)
