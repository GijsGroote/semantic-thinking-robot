from robot_brain.global_variables import PROJECT_PATH
from helper_functions.figures import (
        create_time_plot,
        create_prediction_error_plot,
        create_full_prediction_error_plot,
        create_execution_time_two_runs,
        create_time_plot_two_runs,
        create_hyp_vs_subtask_plot,
        )

def main():
    create_time_plot_two_runs(
            data_path_with_kgraph=PROJECT_PATH+"environments/random_env/data/push_data_kgraph",
            data_path_no_kgraph=PROJECT_PATH+"environments/random_env/data/push_data_no_kgraph")

    # create_multiple_run_time_plot(data_path=PROJECT_PATH+"environments/random_env/data/drive_data_no_kgraph")
    # create_time_plot(data_path=PROJECT_PATH+"environments/random_env/data/drive_data_no_kgraph")
    # create_multiple_run_prediction_error_plot(data_path=PROJECT_PATH+"environments/random_env/data/push_data_kgraph")

if __name__ == '__main__':
    main()
