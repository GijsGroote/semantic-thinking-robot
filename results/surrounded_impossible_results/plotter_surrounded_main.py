
from robot_brain.global_variables import PROJECT_PATH
from helper_functions.figures import create_time_plot, create_prediction_error_plot

def main():
    create_time_plot(data_path=PROJECT_PATH+"logger/logs/*")

if __name__ == '__main__':
    main()
