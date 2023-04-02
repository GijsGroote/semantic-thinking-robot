from plotly.subplots import make_subplots
import json
import plotly.graph_objects as go

from robot_brain.global_variables import PROJECT_PATH

def main():

    total_time = []
    execute_time = []
    search_time = []
    subtask_number = []
    
    # collect date from json files
    for i in range(1, 9):
        subtask_number.append(i)
        # Opening JSON file
        with open(PROJECT_PATH+'images/results/result_surrounded_impossible/task_log_'+str(i)+'.json') as json_file:
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

    fig.update_xaxes(title_text="Taks number", row=1, col=1)
    fig.update_yaxes(title_text="Time [sec]", row=1, col=1)

    fig.update_layout({"title": {"text": "Solving the Impossible Surrounded Task"}})

    # fig.update_layout(paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

    fig.show()

if __name__ == '__main__':
    main()
 
