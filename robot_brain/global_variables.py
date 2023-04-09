DT = 0.05
# dashboard global variables
CREATE_SERVER_DASHBOARD = True
PLOT_CONTROLLER = False
LOG_METRICS = True
DASHBOARD_PORT_PID = 8040

# plot_controller should be True for SAVE_LOG_METRICS
SAVE_LOG_METRICS = True

MIN_INPUT = -5.7
MAX_INPUT = 5.7

UNKNOWN_OBSTACLE_COST = 3.5
KNOWN_OBSTACLE_COST = 3.5
GRID_X_SIZE =12# up to down length
GRID_Y_SIZE =12# left to right length

def in_grid(x:float, y:float) -> bool:
    """ return True if the (x,y) position in
    inside the grid boundaries, otherwise False. """
    return abs(x)<=GRID_X_SIZE/2 and abs(y)<=GRID_Y_SIZE

POINT_ROBOT_RADIUS = 0.35
BOXER_ROBOT_LENGTH = 0.85
BOXER_ROBOT_WIDTH = 0.6

# explore factor between 0 (not learning) and 1 (maximal learning)
EXPLORE_FACTOR = 0.3

# the number of time steps to plot for any controller
PLOT_N_TIMESTEPS = 200

FIG_BG_COLOR = "rgb(230, 230, 255)"  # myEvenLighterColor "rgba(229,236,246,255)"
FIG_BG_COLOR = "rgb(255, 255, 255)"  # white

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
