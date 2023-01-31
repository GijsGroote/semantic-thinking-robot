DT = 0.05
# dashboard global variables
CREATE_SERVER_DASHBOARD = True
PLOT_CONTROLLER = False
LOG_METRICS = True
DASHBOARD_PORT_PID = 8040

# plot_controller should be True for SAVE_LOG_METRICS
SAVE_LOG_METRICS = False

MIN_INPUT = -2.0
MAX_INPUT = 2.0

UNKNOWN_OBSTACLE_COST = 0.5
KNOWN_OBSTACLE_COST = 0.5
GRID_X_SIZE = 10 # up to down length
GRID_Y_SIZE = 12 # left to right length

POINT_ROBOT_RADIUS = 0.18
BOXER_ROBOT_LENGTH = 0.85
BOXER_ROBOT_WIDTH = 0.6

# the number of time steps to plot for any controller
PLOT_N_TIMESTEPS = 200

FIG_BG_COLOR = "rgb(230, 230, 255)"  # myEvenLighterColor "rgba(229,236,246,255)"

COLORS = ["#09ffff", "#19d3f3", "#e763fa" , "#ab63fa"]

# "rgb(178, 178, 255)" # #b2b2ff myDarkColor
# "rgb(204, 204, 255)" # #ccccff myLightColor
# "rgb(230, 230, 255)" # #e6e6ff myEvenLighterColor


# TODO: set this path automatically
PROJECT_PATH = "/home/gijs/Documents/semantic-thinking-robot/"


# MPC_CONTROLLER = "mpc controller"

import torch
# TODO: mppi can be a mess when it's want to calculate efficient on the gpu
TORCH_DEVICE = torch.device("cpu")
# if torch.cuda.is_available():
#     TORCH_DEVICE = torch.device("cuda:0")
