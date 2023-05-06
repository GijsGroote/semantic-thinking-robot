import glob
import os

from robot_brain.global_variables import PROJECT_PATH
from helper_functions.figures import create_time_plot, create_prediction_error_plot

def main():
    create_time_plot(data_path=PROJECT_PATH+"environments/point_robot/data/*",
        save_path=PROJECT_PATH+"environments/point_robot/time_plot.png")

    create_prediction_error_plot(data_path=PROJECT_PATH+"environments/point_robot/data/*",
        save_path=PROJECT_PATH+"environments/point_robot/pe_plot.png")

    # # remove all old files
    # files = glob.glob(PROJECT_PATH+"logger/logs/*")
    # for file in files:
    #     os.remove(file)

if __name__ == '__main__':
    main()
