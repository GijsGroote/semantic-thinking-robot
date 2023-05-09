import os
import random
import math
import glob
import numpy as np
import json
import plotly.express as px
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
        while os.path.exists(save_path):
            n_directories +=1
            save_path = os.path.join(PROJECT_PATH, dir_path + "/run_" + str(n_directories))

        os.mkdir(save_path)
    return save_path

def get_random_color():
    """ return a random color. """
    return [random.random(), random.random(), random.random(), 1]

##### TIME PLOTS #####
def collect_all_time_data(data_path):
    " Collect data and call create time plot function. """

    run_paths = []
    for entry in os.scandir(data_path):
        if entry.is_dir():
            run_paths.append(entry.path)

    total_time = []
    execute_time = []
    search_time = []
    for n_task in glob.glob(run_paths[0]+"/*"):
        total_time.append([])
        execute_time.append([])
        search_time.append([])


    for run_path in run_paths:
        json_file_paths = glob.glob(run_path+"/*")
        json_file_paths.sort()
        for (i, json_file_path) in enumerate(json_file_paths):
            with open(json_file_path) as json_file:
                data = json.load(json_file)
                total_time[i].append(data["total_time"])
                execute_time[i].append(data["execute_time"])
                search_time[i].append(data["search_time"])

    total_time_avg = []
    total_time_std = []
    execute_time_avg = []
    execute_time_std = []
    search_time_avg = []
    search_time_std = []

    # calulate mean and standart deviation
    for (i, _) in enumerate(total_time):
        total_time_avg.append(np.mean(total_time[i]))
        execute_time_avg.append(np.mean(execute_time[i]))
        search_time_avg.append(np.mean(search_time[i]))

        total_time_std.append(np.std(total_time[i]))
        execute_time_std.append(np.std(execute_time[i]))
        search_time_std.append(np.std(search_time[i]))

    return (total_time_avg,
            execute_time_avg,
            search_time_avg,
            total_time_std,
            execute_time_std,
            search_time_std)

def create_time_plot(data_path):

    (total_time_avg,
     execute_time_avg,
     search_time_avg,
     total_time_std,
     execute_time_std,
     search_time_std) = collect_all_time_data(data_path)

    subtask_number = [i for i in range(len(total_time_avg)+1)]

    print(f"Creating Time plot out of {max(subtask_number)} JSON Files")

    fig = px.scatter()

    fig.append_trace(go.Scatter(
        x=[x - 0.03 for x in subtask_number],
        y=total_time_avg,
        mode='markers',
        marker=dict(size=12, opacity=1.0, line=dict(color='black', width=2)),
        error_y=dict(type='data', array=total_time_std),
        name='Total Time',
        ), row=1, col=1)

    fig.append_trace(go.Scatter(
        x=subtask_number,
        y=search_time_avg,
        mode='markers',
        marker=dict(size=12, opacity=1.0, line=dict(color='black', width=2)),
        error_y=dict(type='data', array=search_time_std),
        name='Search Time',
        ), row=1, col=1)

    fig.append_trace(go.Scatter(
        x=[x + 0.03 for x in subtask_number],
        y=execute_time_avg,
        mode='markers',
        marker=dict(size=12, opacity=1.0, line=dict(color='black', width=2)),
        error_y=dict(type='data', array=execute_time_std),
        name='Execution Time',
        ), row=1, col=1)

    fig.update_xaxes(title_text="Number of Tasks experience", row=1, col=1)
    fig.update_yaxes(title_text="Time [sec]", row=1, col=1)
    fig.update_layout(
            paper_bgcolor='white',
            plot_bgcolor='white',
            xaxis=dict(
                zeroline=False,
                zerolinecolor='black',
                zerolinewidth=1
                ),
            yaxis=dict(
                # range=[0, 50],
                zeroline=True,
                gridcolor='black',
                zerolinecolor='black',
                zerolinewidth=1
                )
            )

    fig.show()

def create_time_plot_two_runs(data_path_with_kgraph: str, data_path_no_kgraph: str):

    (total_time_avg_kgraph,_,_,total_time_std_kgraph,_,_) = collect_all_time_data(data_path_with_kgraph)
    (total_time_avg_no_kgraph,_,_,total_time_std_no_kgraph,_,_) = collect_all_time_data(data_path_no_kgraph)

    subtask_number = [i for i in range(len(total_time_avg_kgraph)+1)]

    fig = px.scatter()

    fig.append_trace(go.Scatter(
        x=[x + 0.03 for x in subtask_number],
        y=total_time_avg_no_kgraph,
        mode='markers',
        marker=dict(size=12, opacity=1.0, line=dict(color='black', width=2)),
        error_y=dict(type='data', array=total_time_std_no_kgraph),
        name='No KGraph',
        ), row=1, col=1)

    fig.append_trace(go.Scatter(
        x=[x - 0.03 for x in subtask_number],
        y=total_time_avg_kgraph,
        mode='markers',
        marker=dict(size=12, opacity=1.0, line=dict(color='black', width=2)),
        error_y=dict(type='data', array=total_time_std_kgraph),
        name='With KGraph',
        ), row=1, col=1)

    fig.update_xaxes(title_text="Task number", row=1, col=1)
    fig.update_yaxes(title_text="Time [sec]", row=1, col=1)

    fig.update_layout(
            paper_bgcolor='white',
            plot_bgcolor='white',
            xaxis=dict(
                zeroline=False,
                zerolinecolor='black',
                zerolinewidth=1
                ),
            yaxis=dict(
                # range=[0, 50],
                zeroline=True,
                gridcolor='black',
                zerolinecolor='black',
                zerolinewidth=1
                )
            )  # Set the color of the x-axis gridlines
    fig.show()


##### PREDICTION ERROR #####
def create_multiple_run_prediction_error_plot(data_path):
    run_paths = []
    for entry in os.scandir(data_path):
        if entry.is_dir():
            run_paths.append(entry.path)

    prediction_errors = []
    for n_task in glob.glob(run_paths[0]+"/*"):
        prediction_errors.append([])

    # loop over every run
    for run_path in run_paths:
        # collect date from json files
        json_file_paths = glob.glob(run_path+"/*")
        json_file_paths.sort()

        for (i, json_file_path) in enumerate(json_file_paths):
            # Opening JSON file
            with open(json_file_path) as json_file:
                data = json.load(json_file)

                temp_pred_error = []
                for temp_subtask in data["subtasks"].values():
                    for temp_hypothesis in temp_subtask["hypotheses"].values():
                        for temp_edge in temp_hypothesis["edges"].values():
                            if 'pred_error' in temp_edge:# and isinstance(temp_edge, DriveActionEdge):
                                temp_pred_error = temp_pred_error+temp_edge["pred_error"]


                if not math.isnan(np.mean(temp_pred_error)):
                    prediction_errors[i].append(np.mean(temp_pred_error))
                else:
                    print(f"found a nan!")

    pred_error_avg = []
    pred_error_std = []
    subtask_number = []

    for (i, avg) in enumerate(prediction_errors):
        pred_error_avg.append(np.mean(avg))
        pred_error_std.append(np.std(avg))
        subtask_number.append(i)

    print(f"subtasks number list {subtask_number}")

    fig = go.Figure()

    # add the error bars for the standard deviation
    fig.add_trace(go.Scatter(
        x=subtask_number, y=pred_error_avg, mode='markers', marker=dict(
            size=12, opacity=1.0, line=dict(color='black', width=2)),
            error_y=dict(type='data', array=pred_error_std), showlegend=False))

    # customize the layout
    fig.update_layout(
                      xaxis_title='Number of Tasks experience',
                      yaxis_title='Average Prediction Error and standard deviation',
                      paper_bgcolor='white',
                      plot_bgcolor='white',
                      )

    fig.show()

def create_prediction_error_plot(data_path: str):

    data_path = data_path+"/*"

    pred_error = []

    # collect date from json files
    json_file_paths = glob.glob(data_path)
    json_file_paths.sort()

    for (i, json_file_path) in enumerate(json_file_paths):

        # Opening JSON file
        with open(json_file_path) as json_file:
            data = json.load(json_file)
            pred_error.append(data["total_avg_prediction_error"])


    # throw away the first data point
    # pred_error = pred_error[1:]
    # subtask_number = subtask_number[1:]


def pe_plot(pred_error):
    """ pred_error is a list that contains (at most 10) average prediction errors. """
    assert len(pred_error) <= 10, f"I said at most 10, you've got {len(pred_error)}"

    subtask_number = range(len(pred_error))

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

    fig.show()


def create_full_prediction_error_plot(data_path: str):
    # TOOD: merge full pred errorr with the pe plot above

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
    fig.update_layout(
                      xaxis_title='Number of Tasks experience',
                      yaxis_title='Average Prediction Error and standard deviation',
                      paper_bgcolor='white',
                      plot_bgcolor='white',
                      )

    fig.show()

def create_execution_time_two_runs(data_path_r0: str, data_path_r1: str):

    data_path_r0 = data_path_r0+"/*"
    data_path_r1 = data_path_r1+"/*"

    subtask_number = []
    # collect date from json files
    json_file_r0_paths = glob.glob(data_path_r0)
    json_file_r0_paths.sort()

    execution_time_r0 = np.zeros(len(json_file_r0_paths))
    mpc_r0 = np.zeros(len(json_file_r0_paths))
    mppi_r0 = np.zeros(len(json_file_r0_paths))

    json_file_r1_paths = glob.glob(data_path_r1)
    json_file_r1_paths.sort()

    execution_time_r1 = np.zeros(len(json_file_r1_paths))
    mpc_r1 = np.zeros(len(json_file_r1_paths))
    mppi_r1 = np.zeros(len(json_file_r1_paths))



    for (i, json_file_r0_path) in enumerate(json_file_r0_paths):
        subtask_number.append(i)

        with open(json_file_r0_path) as json_file:
            data = json.load(json_file)
            temp_pred_error = []
            for temp_subtask in data["subtasks"].values():
                for temp_hypothesis in temp_subtask["hypotheses"].values():

                    execution_time_r0[i] += temp_hypothesis['execute_time']
                    for temp_edge in temp_hypothesis["edges"].values():
                        if 'controller_type' in temp_edge:
                            if temp_edge['controller_type'] == "MPC_1th_order":
                                mpc_r0[i] += 1
                            elif temp_edge['controller_type'] == "MPPI":
                                mppi_r0[i] += 1


    for (i, json_file_r1_path) in enumerate(json_file_r1_paths):
        subtask_number.append(i)

        with open(json_file_r1_path) as json_file:
            data = json.load(json_file)
            temp_pred_error = []
            for temp_subtask in data["subtasks"].values():
                for temp_hypothesis in temp_subtask["hypotheses"].values():

                    execution_time_r1[i] += temp_hypothesis['execute_time']
                    for temp_edge in temp_hypothesis["edges"].values():
                        if 'controller_type' in temp_edge:
                            if temp_edge['controller_type'] == "MPC_1th_order":
                                mpc_r1[i] += 0
                            elif temp_edge['controller_type'] == "MPPI":
                                mppi_r1[i] += 0

    fig = go.Figure()

    print(f"for Run 0, mpc: {mpc_r0}, mppi: {mppi_r0}")
    print(f"For run 1, mpc: {mpc_r1}, mppi: {mppi_r1}")

    fig.add_trace(go.Scatter(
        x=subtask_number, y=execution_time_r0, mode='lines', name="with KGraph", showlegend=True))

    fig.add_trace(go.Scatter(
        x=subtask_number, y=execution_time_r1, mode='lines', name="without KGraph", showlegend=True))

    # customize the layout
    fig.update_layout(
                      xaxis_title='Number of Tasks experience',
                      yaxis_title='Execution Time [sec]',
                      paper_bgcolor='white',
                      plot_bgcolor='white'
                      )
    fig.show()


def create_hyp_vs_subtask_plot(data_path: str):

    data_path = data_path+"/*"

    subtask_number = []
    # collect date from json files
    json_file_paths = glob.glob(data_path)
    json_file_paths.sort()

    hyp_and_subtask = np.zeros(len(json_file_paths))

    for (i, json_file_path) in enumerate(json_file_paths):
        subtask_number.append(i)

        with open(json_file_path) as json_file:
            data = json.load(json_file)

            temp_subtask_counter = 0
            temp_hypothesis_counter = 0

            for temp_subtask in data["subtasks"].values():
                temp_subtask_counter += 1
                for temp_hypothesis in temp_subtask["hypotheses"].values():
                    temp_hypothesis_counter += 1

            hyp_and_subtask[i] = temp_hypothesis_counter/temp_subtask_counter

    fig = go.Figure()

    fig.add_trace(go.Scatter(
        x=subtask_number, y=hyp_and_subtask, mode='lines', name="TODOKGraph", showlegend=True))

    # customize the layout
    fig.update_layout(
                      xaxis_title='Number of Tasks experience',
                      yaxis_title='Execution Time [sec]',
                      paper_bgcolor='white',
                      plot_bgcolor='white'
                      )
    fig.show()
