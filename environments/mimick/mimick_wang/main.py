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
    create_time_plot(data_path=PROJECT_PATH+"environments/mimick/mimick_wang/data")

    # display_drive_action_para(data_path=PROJECT_PATH+"environments/mimick/mimick_wang/data")
    # display_push_action_para(data_path=PROJECT_PATH+"environments/mimick/mimick_wang/data")


if __name__ == '__main__':
    main()

