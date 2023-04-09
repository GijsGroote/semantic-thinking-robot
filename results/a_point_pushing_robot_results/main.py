from robot_brain.global_variables import PROJECT_PATH
from results.time_plot import create_time_figure


def main():
    create_time_figure(data_path=PROJECT_PATH+"results/a_point_pushing_robot_results/data/*")

if __name__ == '__main__':
    main()
