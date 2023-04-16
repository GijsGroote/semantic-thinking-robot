import os
import random
import glob
import numpy as np
import json
from plotly.subplots import make_subplots
import plotly.graph_objects as go

from robot_brain.global_variables import PROJECT_PATH, LOG_METRICS, SAVE_LOG_METRICS
from robot_brain.global_planning.hgraph.drive_act_edge import DriveActionEdge


def create_new_directory(dir_path: str) -> str:
    """ create a new directory to save generated files. """
    save_path = None
    if LOG_METRICS and SAVE_LOG_METRICS:
        n_directories = len(next(os.walk(os.path.join(PROJECT_PATH, dir_path)))[1])
        save_path = os.path.join(PROJECT_PATH, dir_path + "/run_" + str(n_directories))
        os.mkdir(save_path)
    return save_path

def get_random_color():
    """ return a random color. """
    return [random.random(), random.random(), random.random(), 1]

def create_time_plot(data_path: str, save_path=False):

    data_path = data_path+"/*"
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

def create_time_plot_two_runs(data_path_with_kgraph: str, data_path_no_kgraph: str, save_path=False):


    data_path_with_kgraph = data_path_with_kgraph+"/*"
    total_time_with_kgraph = []
    execute_time_with_kgraph = []
    search_time_with_kgraph = []
    subtask_number_with_kgraph = []


    data_path_no_kgraph = data_path_no_kgraph+"/*"
    total_time_no_kgraph = []
    execute_time_no_kgraph = []
    search_time_no_kgraph = []
    subtask_number_no_kgraph = []


    # collect date from json files
    json_file_with_kgraph_paths = glob.glob(data_path_with_kgraph)
    json_file_with_kgraph_paths.sort()

    for (i, json_file_path) in enumerate(json_file_with_kgraph_paths):
        subtask_number_with_kgraph.append(i)
        # Opening JSON file
        with open(json_file_path) as json_file:
            data_with_kgraph= json.load(json_file)
            total_time_with_kgraph.append(data_with_kgraph["total_time"])
            execute_time_with_kgraph.append(data_with_kgraph["execute_time"])
            search_time_with_kgraph.append(data_with_kgraph["search_time"])

    # collect date from json files
    json_file_no_kgraph_paths = glob.glob(data_path_with_kgraph)
    json_file_no_kgraph_paths.sort()

    for (i, json_file_path) in enumerate(json_file_no_kgraph_paths):
        subtask_number_no_kgraph.append(i)
        # Opening JSON file
        with open(json_file_path) as json_file:
            data_no_kgraph = json.load(json_file)
            total_time_no_kgraph.append(data_no_kgraph["total_time"])
            execute_time_no_kgraph.append(data_no_kgraph["execute_time"])
            search_time_no_kgraph.append(data_no_kgraph["search_time"])

    assert len(subtask_number_with_kgraph) == len(subtask_number_no_kgraph)
    print(f"one: {len(subtask_number_no_kgraph)} runs, {len(total_time_with_kgraph)} and two {len(total_time_no_kgraph)} ")


    fig = make_subplots(rows=1, cols=1)
    # fig.append_trace(go.Scatter(
    #     x=subtask_number_with_kgraph,
    #     y=total_time_with_kgraph,
    #     name="Total time",
    #     line=dict(color='medium purple', width=1, dash='dash')
    #     ), row=1, col=1)
    # fig.append_trace(go.Scatter(
    #     x=subtask_number_with_kgraph,
    #     y=execute_time_with_kgraph,
    #     name="Execute time",
    #     line=dict(color='forest green', width=1, dash='dash')
    #     ), row=1, col=1)
    fig.append_trace(go.Scatter(
        x=subtask_number_with_kgraph,
        y=execute_time_with_kgraph,
        name="Without Knowledge Graph",
        line=dict(color='forest green', width=1, dash='dash')
        ), row=1, col=1)

    # add number 2 to list
    # fig.append_trace(go.Scatter(
    #     x=subtask_number_no_kgraph,
    #     y=total_time_no_kgraph,
    #     name="Total time",
    #     line=dict(color='medium purple', width=1, dash='dash')
    #     ), row=1, col=1)
    fig.append_trace(go.Scatter(
        x=subtask_number_no_kgraph,
        y=execute_time_no_kgraph,
        name="With Knowledge Graph",
        line=dict(color='forest green', width=1, dash='dash')
        ), row=1, col=1)
    # fig.append_trace(go.Scatter(
    #     x=subtask_number_no_kgraph,
    #     y=search_time_no_kgraph,
    #     name="Search Time",
    #     line=dict(color='forest green', width=1, dash='dash')
    #     ), row=1, col=1)

    fig.update_xaxes(title_text="Task number", row=1, col=1)
    fig.update_yaxes(title_text="Time [sec]", row=1, col=1)

    fig.update_layout({"title": {"text": "Search and Execution times"}})

    # fig.update_layout(paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

    if save_path is False:
        fig.show()
    else:
        fig.write_image(save_path)


def create_prediction_error_plot(data_path: str, save_path=False):

    data_path = data_path+"/*"

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

    # throw away the first data point
    # pred_error = pred_error[1:]
    # subtask_number = subtask_number[1:]

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

def create_full_prediction_error_plot(data_path: str, save_path=False):

    data_path = data_path+"/*"

    pred_error_avg = []
    pred_error_std = []
    subtask_number = []

    # collect date from json files
    json_file_paths = glob.glob(data_path)
    json_file_paths.sort()

    for (i, json_file_path) in enumerate(json_file_paths):
        subtask_number.append(i)
        # Opening JSON file
        with open(json_file_path) as json_file:
            data = json.load(json_file)

            temp_pred_error = []
            for temp_subtask in data["subtasks"].values():
                for temp_hypothesis in temp_subtask["hypotheses"].values():
                    for temp_edge in temp_hypothesis["edges"].values():
                        if 'pred_error' in temp_edge:# and isinstance(temp_edge, DriveActionEdge):
                            temp_pred_error = temp_pred_error+temp_edge["pred_error"]


            pred_error_avg.append(np.mean(temp_pred_error))
            pred_error_std.append(np.std(temp_pred_error))

    fig = go.Figure()

    # add the error bars for the standard deviation
    fig.add_trace(go.Scatter(
        x=subtask_number, y=pred_error_avg, mode='markers', marker=dict(
            size=12, opacity=1.0, line=dict(color='black', width=2)),
            error_y=dict(type='data', array=pred_error_std), showlegend=False))

    # customize the layout
    fig.update_layout(title='Average Prediction Error for a Repeated Task',
                      xaxis_title='Number of Tasks experience',
                      yaxis_title='Average Prediction Error and standard deviation')

    # fig.add_trace(
    #     go.Scatter(x=subtask_number, y=np.add(
    #         pred_error_avg, pred_error_std),
    #         fill='tonexty', fillcolor='rgba(0,100,80,0.2)',
    #         line_color='rgba(255,255,255,0)', showlegend=False)
    # )
    # fig.add_trace(
    #     go.Scatter(x=subtask_number, y=np.subtract(pred_error_avg, pred_error_std),
    #         fill='tonexty', fillcolor='rgba(0,176,246,0.2)',
    #         line_color='rgba(255,255,255,0)', showlegend=False)
    # )
    #

    if save_path is False:
        fig.show()
    else:
        fig.write_image(save_path)

def create_execution_time_two_runs(data_path_r1: str, data_path_r2: str, save_path=False):

    data_path_r1 = data_path_r1+"/*"
    data_path_r2 = data_path_r2+"/*"

    subtask_number = []
    execution_time_r1 = []
    mpc_r1 = []
    mppi_r1 = []

    execution_time_r2 = []
    mpc_r2 = []
    mppi_r2 = []

    # collect date from json files
    json_file_r1_paths = glob.glob(data_path_r1)
    json_file_r1_paths.sort()

    json_file_r2_paths = glob.glob(data_path_r2)
    json_file_r2_paths.sort()

    print(f"data r1 {json_file_r1_paths}")

    for (i, json_file_r1_path) in enumerate(json_file_r1_paths):
        subtask_number.append(i)

        with open(json_file_r1_path) as json_file:
            data = json.load(json_file)


            temp_pred_error = []
            for temp_subtask in data["subtasks"].values():
                for temp_hypothesis in temp_subtask["hypotheses"].values():

                    for temp_edge in temp_hypothesis["edges"].values():

                        print(temp_edge.keys())


                        print(f'temp_hypothesis {temp_edge["status"]}')
                        # todo this should have more ifs
                        if 'execute_time' in temp_edge:# and isinstance(temp_edge, DriveActionEdge):

                            execution_time_r1[i] += temp_edge['execution_time']

                            if temp_edge['controller_type'] == "MPC":
                                print(f"mpc detected now at {mppi_r1}")
                                mpc_r1[i] += 1
                            elif temp_edge['controller_type'] == "MPPI":

                                mppi_r1[i] += 1
                                print(f"mppi detected now at {mppi_r1}")


    fig = go.Figure()

    # add the error bars for the standard deviation
    fig.add_trace(go.Scatter(
        x=subtask_number, y=execution_time_r1, mode='markers', showlegend=False))

    # customize the layout
    fig.update_layout(title='Execution times for a Repeated Task',
                      xaxis_title='Number of Tasks experience',
                      yaxis_title='Execution Time [sec]')


    if save_path is False:
        fig.show()
    else:
        fig.write_image(save_path)
