import os
import pandas as pd
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


def collect_data(data_path):
    " Collect all data in panda's dataframe. """

    run_paths = []
    for entry in os.scandir(data_path):
        if entry.is_dir():
            run_paths.append(entry.path)

    total_time = []
    execute_time = []
    search_time = []
    drive_pe = []
    push_pe = []

    for n_task in glob.glob(run_paths[0]+"/*"):
        total_time.append([])
        execute_time.append([])
        search_time.append([])
        drive_pe.append([])
        push_pe.append([])

    for run_path in run_paths:
        json_file_paths = glob.glob(run_path+"/*")
        json_file_paths.sort()
        for (i, json_file_path) in enumerate(json_file_paths):
            with open(json_file_path) as json_file:
                data = json.load(json_file)
                total_time[i].append(data["total_time"])
                execute_time[i].append(data["execute_time"])
                search_time[i].append(data["search_time"])


                for temp_subtask in data["subtasks"].values():
                    for temp_hypothesis in temp_subtask["hypotheses"].values():
                        for temp_edge in temp_hypothesis["edges"].values():
                            if 'pred_error' in temp_edge:# and isinstance(temp_edge, DriveActionEdge):
                                if temp_edge["verb"] == "driving":
                                    drive_pe[i] = drive_pe[i]+temp_edge["pred_error"]

                                if temp_edge["verb"] == "pushing":
                                    push_pe[i] = push_pe[i] + temp_edge["pred_error"]


    n_subtasks = len(total_time)
    print(f"number of subtasks in a task: {n_subtasks}")

    data = {
            "total_time_median": [np.median(total_time[i]) for i in range(n_subtasks)],
            "total_time_mean": [np.mean(total_time[i]) for i in range(n_subtasks)],
            "search_time_mean": [np.mean(search_time[i]) for i in range(n_subtasks)],
            "execute_time_mean": [np.mean(execute_time[i]) for i in range(n_subtasks)],
            "total_time_std": [np.std(total_time[i]) for i in range(n_subtasks)],
            "search_time_std": [np.std(search_time[i]) for i in range(n_subtasks)],
            "execute_time_std": [np.std(execute_time[i]) for i in range(n_subtasks)],
            "total_time": total_time,
            "search_time": search_time,
            "execute_time": execute_time,
            "drive_pe": drive_pe,
            "drive_pe_mean": [np.mean(drive_pe[i]) for i in range(n_subtasks)],
            "push_pe": push_pe,
            "push_pe_mean": [np.mean(push_pe[i]) for i in range(n_subtasks)],
            }


    return pd.DataFrame(data)


##### TIME PLOTS #####
def create_time_plot(data_path):

    df = collect_data(data_path)

    fig = make_subplots(rows=1, cols=1)

    showlegend = True
    for i in range(len(df)):
        fig.add_trace(go.Box(
            y=df['total_time'][i],
            x=[i-0.1]*len(df['total_time'][i]),
            fillcolor='#51BBE2',
            marker=dict(
                size=2,
                color='rgb(0, 0, 0)'
            ),
            showlegend=showlegend,
            name="total time",
            legendgroup='1',
            ), row=1, col=1,
            )

        fig.add_trace(go.Box(
            y=df['execute_time'][i],
            x=[i+0.1]*len(df['execute_time'][i]),
            legendgroup='2',
            fillcolor='#8AC15A',
            marker=dict(
                size=2,
                color='rgb(0, 0, 0)'
            ),
            name="execute time",

            showlegend=showlegend,
            ), row=1, col=1,
            )

        fig.add_trace(go.Box(
            y=df['search_time'][i],
            x=[i]*len(df['search_time'][i]),
            fillcolor='#FFD55E',
            marker=dict(
                size=2,
                color='rgb(0, 0, 0)'
            ),
            name="search time",

            showlegend=showlegend,
            legendgroup='3',
            ), row=1, col=1,
            )

        showlegend=False

    fig.update_traces(width=0.10)

    # customize the layout
    fig.update_layout(
            # yaxis_range=[0, 100],
            xaxis_title='Number of Tasks experience',
            yaxis_title='Time [sec]',
            # yaxis_type='log',
            paper_bgcolor='white',
            plot_bgcolor='white',
            xaxis=dict(
                zeroline=False,
                zerolinecolor='black',
                zerolinewidth=1
                ),
            yaxis=dict(
                range=[0, 46],

                zeroline=True,
                gridcolor='black',
                zerolinecolor='black',
                zerolinewidth=1
                ),
            showlegend=True,

            )

    fig.show()


def create_time_with_without_kgraph_plot(data_path_kgraph: str, data_path_no_kgraph: str):

    kgraph_df = collect_data(data_path_kgraph)
    no_kgraph_df = collect_data(data_path_no_kgraph)

    subtask_number = [i for i in range(len(no_kgraph_df)+1)]

    fig = px.scatter()

    fig.append_trace(go.Scatter(
        x=[x for x in subtask_number],
        # y=no_kgraph_df["total_time_mean"],
        y=no_kgraph_df["total_time_median"],
        mode='markers',
        marker=dict(size=12, opacity=1.0, line=dict(color='black', width=2)),
        # error_y=dict(type='data', array=total_time_std_no_kgraph),
        name='No KGraph',
        ), row=1, col=1)

    fig.append_trace(go.Scatter(
        x=[x for x in subtask_number],
        # y=kgraph_df["total_time_mean"],
        y=kgraph_df["total_time_median"],
        mode='markers',
        marker=dict(size=12, opacity=1.0, line=dict(color='black', width=2)),
        # error_y=dict(type='data', array=total_time_std_kgraph),
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
def create_drive_pe_with_without_kgraph_plot(data_path_kgraph: str, data_path_no_kgraph: str):

    kgraph_df = collect_data(data_path_kgraph)
    no_kgraph_df = collect_data(data_path_no_kgraph)

    fig = make_subplots(rows=1, cols=1)

    showlegend = True
    for i in range(len(no_kgraph_df)):


        fig.add_trace(go.Box(
            y=kgraph_df['drive_pe'][i],
            x=[i-0.1]*len(kgraph_df['drive_pe'][i]),
            fillcolor='#b2b2ff',
            marker=dict(
                size=2,
                color='rgb(0, 0, 0)'
            ),
            showlegend=showlegend,
            name="with kgraph",
            legendgroup='1',
            ), row=1, col=1,
            )

        fig.add_trace(go.Box(
            y=no_kgraph_df['drive_pe'][i],
            x=[i+0.1]*len(no_kgraph_df['drive_pe'][i]),
            legendgroup='2',
            fillcolor='#FFD55E',
            marker=dict(
                size=2,
                color='rgb(0, 0, 0)'
            ),
            name="no kgraph",

            showlegend=showlegend,
            ), row=1, col=1,
            )
        showlegend=False

    fig.update_traces(width=0.07)

    # customize the layout
    fig.update_layout(
            xaxis_title='Number of Tasks experience',
            yaxis_title='Prediction Error [m]',
            # yaxis_type='log',
            paper_bgcolor='white',
            plot_bgcolor='white',
            xaxis=dict(
                zeroline=False,
                zerolinecolor='black',
                zerolinewidth=1
                ),
            yaxis=dict(
                zeroline=True,
                gridcolor='black',
                zerolinecolor='black',
                zerolinewidth=1
                ),
            showlegend=True,

            )

    fig.show()


def create_push_pe_with_without_kgraph_plot(data_path_kgraph: str, data_path_no_kgraph: str):

    kgraph_df = collect_data(data_path_kgraph)
    no_kgraph_df = collect_data(data_path_no_kgraph)

    fig = make_subplots(rows=1, cols=1)

    showlegend = True
    for i in range(len(no_kgraph_df)):

        fig.add_trace(go.Box(
            y=kgraph_df['push_pe'][i],
            x=[i-0.1]*len(kgraph_df['push_pe'][i]),
            fillcolor='#b2b2ff',
            marker=dict(
                size=2,
                color='rgb(0, 0, 0)',
            ),
            showlegend=showlegend,
            name="with kgraph",
            legendgroup='1',
            ), row=1, col=1,
            )

        fig.add_trace(go.Box(
            y=no_kgraph_df['push_pe'][i],
            x=[i+0.1]*len(no_kgraph_df['push_pe'][i]),
            legendgroup='2',
            fillcolor='#FFD55E',
            marker=dict(
                size=2,
                color='rgb(0, 0, 0)'
            ),
            name="no kgraph",

            showlegend=showlegend,
            ), row=1, col=1,
            )
        showlegend=False

    fig.update_traces(width=0.07)

    # customize the layout
    fig.update_layout(
            # yaxis_type='log',
            paper_bgcolor='white',
            plot_bgcolor='white',
            xaxis=dict(
                zeroline=False,
                zerolinecolor='black',
                zerolinewidth=1
                ),
            yaxis=dict(
                title='Prediction Error [m]',
                # range=[0, 1],
                zeroline=True,
                gridcolor='black',
                zerolinecolor='black',
                zerolinewidth=1
                ),
            showlegend=True,

            )

    fig.show()

def create_pe_two_runs(data_path_with_kgraph: str, data_path_no_kgraph: str):
    pass
    # kgraph_df =  collect_data(data_path_with_kgraph)
    # no_kgraph_df =  collect_data(data_path_no_kgraph)

    # fig = go.Figure()

    # # add the error bars for the standard deviation
    # fig.add_trace(go.Scatter(
    #     x=subtask_number, y=pred_error_avg, mode='markers', marker=dict(
    #         size=12, opacity=1.0, line=dict(color='black', width=2)),
    #         error_y=dict(type='data', array=pred_error_std), showlegend=False))

    # # customize the layout
    # fig.update_layout(
    #                   xaxis_title='Number of Tasks experience',
    #                   yaxis_title='Average Prediction Error and standard deviation',
    #                   paper_bgcolor='white',
    #                   plot_bgcolor='white',
    #                   )

    # fig.show()

    # print(f"for Run 0, mpc: {mpc_r0}, mppi: {mppi_r0}")
    # print(f"For run 1, mpc: {mpc_r1}, mppi: {mppi_r1}")

    # fig.add_trace(go.Scatter(
    #     x=subtask_number, y=execution_time_r0, mode='lines', name="with KGraph", showlegend=True))

    # fig.add_trace(go.Scatter(
    #     x=subtask_number, y=execution_time_r1, mode='lines', name="without KGraph", showlegend=True))

    # # customize the layout
    # fig.update_layout(
    #                   xaxis_title='Number of Tasks experience',
    #                   yaxis_title='Execution Time [sec]',
    #                   paper_bgcolor='white',
    #                   plot_bgcolor='white'
    #                   )
    # fig.show()


def create_hyp_vs_subtask_plot(data_path: str):
    pass

    # data_path = data_path+"/*"

    # subtask_number = []
    # # collect date from json files
    # json_file_paths = glob.glob(data_path)
    # json_file_paths.sort()

    # hyp_and_subtask = np.zeros(len(json_file_paths))

    # for (i, json_file_path) in enumerate(json_file_paths):
    #     subtask_number.append(i)

    #     with open(json_file_path) as json_file:
    #         data = json.load(json_file)

    #         temp_subtask_counter = 0
    #         temp_hypothesis_counter = 0

    #         for temp_subtask in data["subtasks"].values():
    #             temp_subtask_counter += 1
    #             for temp_hypothesis in temp_subtask["hypotheses"].values():
    #                 temp_hypothesis_counter += 1

    #         hyp_and_subtask[i] = temp_hypothesis_counter/temp_subtask_counter

    # fig = go.Figure()

    # fig.add_trace(go.Scatter(
    #     x=subtask_number, y=hyp_and_subtask, mode='lines', name="TODOKGraph", showlegend=True))

    # # customize the layout
    # fig.update_layout(
    #                   xaxis_title='Number of Tasks experience',
    #                   yaxis_title='Execution Time [sec]',
    #                   paper_bgcolor='white',
    #                   plot_bgcolor='white'
    #                   )
    # fig.show()

##### TABLES #####
def display_drive_action_para(data_path):
    """ Display a table with the parameterisations of drive actions. """

    para_dict = find_all_existing_para(data_path)

    run_paths = []
    for entry in os.scandir(data_path):
        if entry.is_dir():
            run_paths.append(entry.path)

    for run_path in run_paths:
        json_file_paths = glob.glob(run_path+"/*")
        json_file_paths.sort()

        for (i, json_file_path) in enumerate(json_file_paths):
            with open(json_file_path) as json_file:
                data = json.load(json_file)
                for temp_subtask in data["subtasks"].values():
                    for temp_hypothesis in temp_subtask["hypotheses"].values():
                        for temp_edge in temp_hypothesis["edges"].values():
                            if 'controller_type' in temp_edge:
                                para_dict[temp_edge['controller_type']][i] += 1


    drive_para_dict = {}
    if 'MPC_2th_order' in para_dict:
        drive_para_dict['mpc'] = para_dict['MPC_2th_order']
    if 'MPPI' in para_dict:
        drive_para_dict['mppi'] = para_dict['MPPI']

    print(f"drive edge para's {drive_para_dict}")


    for i in range(len(next(iter(drive_para_dict.values())))):
        added = 0

        if 'mpc' in drive_para_dict:
            added += drive_para_dict['mpc'][i]
        if 'mppi' in drive_para_dict:
            added += drive_para_dict['mppi'][i]

        if 'mpc' in drive_para_dict:
            drive_para_dict['mpc'][i] = round(drive_para_dict['mpc'][i]/added, 2)
        if 'mppi' in drive_para_dict:
            drive_para_dict['mppi'][i] = round(drive_para_dict['mppi'][i]/added, 2)

    print(f"drive edge para's [normalized] {drive_para_dict}")



def display_push_action_para(data_path: str):
    """ Display a table with the parameterisations of drive actions. """

    para_dict = find_all_existing_para(data_path)

    run_paths = []
    for entry in os.scandir(data_path):
        if entry.is_dir():
            run_paths.append(entry.path)

    for run_path in run_paths:
        json_file_paths = glob.glob(run_path+"/*")
        json_file_paths.sort()

        for (i, json_file_path) in enumerate(json_file_paths):
            with open(json_file_path) as json_file:
                data = json.load(json_file)
                for temp_subtask in data["subtasks"].values():
                    for temp_hypothesis in temp_subtask["hypotheses"].values():
                        for temp_edge in temp_hypothesis["edges"].values():
                            if 'controller_type' in temp_edge:
                                para_dict[temp_edge['controller_type']][i] += 1



    push_para_dict = {}
    if 'MPPI_4th_order' in para_dict:
        push_para_dict['mppi_4'] = para_dict['MPPI_4th_order']
    if 'MPPI_5th_order' in para_dict:
        push_para_dict['mppi_5'] = para_dict['MPPI_5th_order']

    print(f"push edge para's {push_para_dict}")

    for i in range(len(next(iter(push_para_dict.values())))):
        added = 0
        if 'mppi_4' in para_dict:
            added += push_para_dict['mppi_4'][i]
        if 'mppi_5' in para_dict:
            added += push_para_dict['mppi_5'][i]

        if 'mppi_4' in para_dict:
            push_para_dict['mppi_4'][i] = round(push_para_dict['mppi_4'][i]/added, 2)
        if 'mppi_5' in para_dict:
            push_para_dict['mppi_5'][i] = round(push_para_dict['mppi_5'][i]/added, 2)

    print(f"push edge para's [normalized] {push_para_dict}")


def find_all_existing_para(data_path: str) -> dict:
    """ collect all existing parameterisations from JSON files.
    return dictionary with keys the parameterizations, and value a list with n empty lists inside
    n corresponding to the number of tasks in a run.
    """

    run_paths = []
    for entry in os.scandir(data_path):
        if entry.is_dir():
            run_paths.append(entry.path)

    # find all drive edge param
    edge_param = set()


    n = None

    for run_path in run_paths:
        json_file_paths = glob.glob(run_path+"/*")
        json_file_paths.sort()

        for (i, json_file_path) in enumerate(json_file_paths):

            if n is None:
                n = len(json_file_paths)

            with open(json_file_path) as json_file:
                data = json.load(json_file)
                for temp_subtask in data["subtasks"].values():
                    for temp_hypothesis in temp_subtask["hypotheses"].values():
                        for temp_edge in temp_hypothesis["edges"].values():
                            if 'controller_type' in temp_edge:
                                edge_param.add(temp_edge['controller_type'])

    edge_param_dict = {}
    for param in edge_param:
        edge_param_dict[param] = [0] * n

    return edge_param_dict

