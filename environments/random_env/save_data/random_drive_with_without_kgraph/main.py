# data generated in run_0 and run_1 was done with seed = 14
from robot_brain.global_variables import PROJECT_PATH
from helper_functions.figures import (
        create_time_plot,
        create_prediction_error_plot,
        create_full_prediction_error_plot,
        create_execution_time_two_runs,
        create_time_plot_two_runs,
        )


def main():
    data_path_0 = PROJECT_PATH+"environments/random_env/save_data/random_drive_with_without_kgraph/run_0"
    data_path_1 = PROJECT_PATH+"environments/random_env/save_data/random_drive_with_without_kgraph/run_1"

    # create_time_plot(data_path=data_path_1)
    # create_prediction_error_plot(data_path=data_path_1)
    # create_full_prediction_error_plot(data_path=data_path_1)
    #
    create_execution_time_two_runs(
            data_path_r0=data_path_1,
            data_path_r1=data_path_0
            )

if __name__ == '__main__':
    main()
