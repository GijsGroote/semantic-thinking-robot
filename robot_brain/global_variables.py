DT = 0.05
# dashboard global variables
CREATE_SERVER_DASHBOARD = True
PLOT_CONTROLLER = False
LOG_METRICS = False
DASHBOARD_PORT_PID = 8040

SAVE_LOG_METRICS = False

MIN_INPUT = -1
MAX_INPUT = 1

UNKNOWN_OBSTACLE_COST = 3.5
KNOWN_OBSTACLE_COST = 2.0
GRID_X_SIZE =12# up to down length
GRID_Y_SIZE =12# left to right length

POINT_ROBOT_RADIUS = 0.35
BOXER_ROBOT_LENGTH = 0.85
BOXER_ROBOT_WIDTH = 0.6

# the number of time steps to plot for any controller
PLOT_N_TIMESTEPS = 200

FIG_BG_COLOR = "rgb(230, 230, 255)"  # myEvenLighterColor "rgba(229,236,246,255)"
FIG_BG_COLOR = "rgb(255, 255, 255)"  # white

COLORS = ["#09ffff", "#19d3f3", "#e763fa" , "#ab63fa"]


PROJECT_PATH = "/home/gijs/Documents/semantic-thinking-robot/"
DATA_PATH = None

import torch
# TODO: mppi can be a mess when it's want to calculate efficient on the gpu
TORCH_DEVICE = torch.device("cpu")
# if torch.cuda.is_available():
#     TORCH_DEVICE = torch.device("cuda:0")
