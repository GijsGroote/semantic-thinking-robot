import random
import glob
import json
from plotly.subplots import make_subplots
import plotly.graph_objects as go

import numpy as np

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
                        if 'pred_error' in temp_edge:
                            temp_pred_error = temp_pred_error+temp_edge["pred_error"]


            pred_error_avg.append(np.mean(temp_pred_error))
            pred_error_std.append(np.std(temp_pred_error))

    # print(f"pred avg {pred_error_avg}, means {pred_error_std}")
    # print(f"Creating prediction error plot out of {max(subtask_number)+1} JSON Files")


    # create the plot
    fig = go.Figure()

    # add the mean markers and lines
    fig.add_trace(go.Scatter(x=subtask_number, y=pred_error_avg, mode='markers+lines', name='Mean', line_color='black', showlegend=True))


    fig.update_layout(title='Average Prediction Error for a Repeated Task',
                      xaxis_title='Number of Tasks experience',
                      yaxis_title='Average Prediction Error')

    fig.show()

    # add deviation
    fig.add_trace(
        go.Scatter(x=subtask_number, y=np.add(pred_error_avg, pred_error_std), mode='lines', line_color='blue', name='standart deviation', showlegend=True)
    )
    fig.add_trace(
        go.Scatter(x=subtask_number, y=np.subtract(pred_error_avg, pred_error_std), mode='lines', line_color='blue', showlegend=False)
    )
    fig.add_trace(
        go.Scatter(x=subtask_number, y=np.add(pred_error_avg, pred_error_std), fill='tonexty', fillcolor='rgba(0,100,80,0.2)', line_color='rgba(255,255,255,0)', showlegend=False)
    )

    fig.add_trace(
        go.Scatter(x=subtask_number, y=np.subtract(pred_error_avg, pred_error_std), fill='tonexty', fillcolor='rgba(0,176,246,0.2)', line_color='rgba(255,255,255,0)', showlegend=False)
    )

    # customize the layout
    fig.update_layout(title='Average and Standard Deviation of Prediction Error for a Repeated Task',
                      xaxis_title='Lists',
                      yaxis_title='Average and Standard Deviation')
    fig.show()
    # fig = make_subplots(rows=1, cols=1)
    # fig.append_trace(go.Scatter(
    #     x=subtask_number,
    #     y=pred_error,
    #     name="Average Prediction Error",
    #     line=dict(color='medium purple', width=1, dash='dash')
    #     ), row=1, col=1)
    #
    # fig.update_xaxes(title_text="Task number", row=1, col=1)
    # fig.update_yaxes(title_text="Prediction Error", row=1, col=1)
    #
    # fig.update_layout({"title": {"text": "Prediction Error Plot"}})
    #
    # # fig.update_layout(paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

    # if save_path is False:
    #     fig.show()
    # else:
    #     fig.write_image(save_path)


if __name__ == '__main__':
    create_full_prediction_error_plot(data_path="/home/gijs/Documents/semantic-thinking-robot/environments/push_point_env/data/run_5")

