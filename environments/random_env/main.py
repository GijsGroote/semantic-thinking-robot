from robot_brain.global_variables import PROJECT_PATH
from helper_functions.figures import (
        create_time_plot,
        create_time_with_without_kgraph_plot,
        create_drive_pe_with_without_kgraph_plot,
        create_push_pe_with_without_kgraph_plot,
        display_drive_action_para,
        display_push_action_para,
        )


def main():
    # drive task random plots
    create_time_plot(data_path=PROJECT_PATH+"environments/random_env/data/drive_data_kgraph")
    create_time_plot(data_path=PROJECT_PATH+"environments/random_env/data/drive_data_no_kgraph")

    create_time_with_without_kgraph_plot(
            data_path_kgraph=PROJECT_PATH+"environments/random_env/data/drive_data_kgraph",
            data_path_no_kgraph=PROJECT_PATH+"environments/random_env/data/drive_data_no_kgraph")

    # # push task random plots
    create_time_plot(data_path=PROJECT_PATH+"environments/random_env/data/push_data_kgraph")
    create_time_plot(data_path=PROJECT_PATH+"environments/random_env/data/push_data_no_kgraph")
    create_time_with_without_kgraph_plot(
            data_path_kgraph=PROJECT_PATH+"environments/random_env/data/push_data_kgraph",
            data_path_no_kgraph=PROJECT_PATH+"environments/random_env/data/push_data_no_kgraph")

    create_push_pe_with_without_kgraph_plot(
            data_path_kgraph=PROJECT_PATH+"environments/random_env/data/push_data_kgraph",
            data_path_no_kgraph=PROJECT_PATH+"environments/random_env/data/push_data_no_kgraph")


if __name__ == '__main__':
    main()
