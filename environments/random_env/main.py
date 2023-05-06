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
    # create_time_plot(data_path=PROJECT_PATH+"environments/random_env/data/run_16/")
    # create_prediction_error_plot(data_path=PROJECT_PATH+"environments/random_env/save_data/random_drive_with_without_kgraph/run_1/")
    # create_prediction_error_plot(data_path=PROJECT_PATH+"environments/random_env/save_data/random_drive_with_without_kgraph/run_0/")
    # create_full_prediction_error_plot(data_path=PROJECT_PATH+"environments/random_env/save_data/random_drive_with_without_kgraph/run_1/")
    # create_full_prediction_error_plot(data_path=PROJECT_PATH+"environments/random_env/save_data/random_drive_with_without_kgraph/run_0/")
    # create_execution_time_two_runs(
    #         data_path_r0=PROJECT_PATH+"environments/random_env/data/run_21/",
    #         data_path_r1=PROJECT_PATH+"environments/random_env/data/run_15/"
    #        r  )

    create_hyp_vs_subtask_plot(PROJECT_PATH+"environments/random_env/data/run_6/")

if __name__ == '__main__':
    main()
