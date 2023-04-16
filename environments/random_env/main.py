from robot_brain.global_variables import PROJECT_PATH
from helper_functions.figures import (
        create_time_plot,
        create_prediction_error_plot,
        create_full_prediction_error_plot,
        create_execution_time_two_runs,
        create_time_plot_two_runs,
        )


def main():
    # create_time_plot(data_path=PROJECT_PATH+"environments/random_env/data/run_16/")
    # create_prediction_error_plot(data_path=PROJECT_PATH+"environments/random_env/data/run_16/")
    # create_full_prediction_error_plot(data_path=PROJECT_PATH+"environments/random_env/data/run_2/")
    create_execution_time_two_runs(
            data_path_r1=PROJECT_PATH+"environments/random_env/data/run_2/",
            data_path_r2=PROJECT_PATH+"environments/random_env/data/run_4/"
            )

if __name__ == '__main__':
    main()
