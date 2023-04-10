import random
import glob
import json
from plotly.subplots import make_subplots
import plotly.graph_objects as go

def get_random_color():
    """ return a random color. """
    return [random.random(), random.random(), random.random(), 1]


def create_time_plot(data_path: str, save_path=False):

    total_time = []
    execute_time = []
    search_time = []
    subtask_number = []


    # collect date from json files
    json_file_paths = glob.glob(data_path)
    json_file_paths.sort()

    for (i, json_file_path) in enumerate(json_file_paths):
        subtask_number.append(i)
        # Opening JSON file
        with open(json_file_path) as json_file:
            data = json.load(json_file)
            total_time.append(data["total_time"])
            execute_time.append(data["execute_time"])
            search_time.append(data["search_time"])

    print(f"Creating Time plot out of {max(subtask_number)} JSON Files")

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

def create_prediction_error_plot(data_path: str, save_path=False):

    pred_error = []
    subtask_number = []

    # collect date from json files
    json_file_paths = glob.glob(data_path)
    json_file_paths.sort()

    for (i, json_file_path) in enumerate(json_file_paths):
        subtask_number.append(i)
        # Opening JSON file
        with open(json_file_path) as json_file:
            data = json.load(json_file)
            pred_error.append(data["total_avg_prediction_error"])

    print(f"Creating prediction error plot out of {max(subtask_number)} JSON Files")

    fig = make_subplots(rows=1, cols=1)
    fig.append_trace(go.Scatter(
        x=subtask_number,
        y=pred_error,
        name="Average Prediction Error",
        line=dict(color='medium purple', width=1, dash='dash')
        ), row=1, col=1)

    fig.update_xaxes(title_text="Task number", row=1, col=1)
    fig.update_yaxes(title_text="Prediction Error", row=1, col=1)

    fig.update_layout({"title": {"text": "Prediction Error Plot"}})

    # fig.update_layout(paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

    if save_path is False:
        fig.show()
    else:
        fig.write_image(save_path)
