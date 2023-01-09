DT = 0.05

# dashboard global variables
CREATE_SERVER_DASHBOARD = False#True
PLOT_CONTROLLER = False
LOG_METRICS = False

# plot_controller should be True for SAVE_LOG_METRICS
SAVE_LOG_METRICS = False

# you could add a rotation constraints here as well
MIN_INPUT = -2.0
MAX_INPUT = 2.0


# the number of time steps to plot for any controller
PLOT_N_TIMESTEPS = 200

FIG_BG_COLOR = "rgb(230, 230, 255)"  # myEvenLighterColor "rgba(229,236,246,255)"

COLORS = ["#09ffff", "#19d3f3", "#e763fa" , "#ab63fa"]

# "rgb(178, 178, 255)" # #b2b2ff myDarkColor
# "rgb(204, 204, 255)" # #ccccff myLightColor
# "rgb(230, 230, 255)" # #e6e6ff myEvenLighterColor


# TODO: set this path automatically
PROJECT_PATH = "/home/gijs/Documents/semantic-thinking-robot/"

UNKNOWN_OBSTACLE_COST = 150
KNOWN_OBSTACLE_COST = 100

# MPC_CONTROLLER = "mpc controller"

import torch
# TODO: mppi can be a mess when it's want to calculate efficient on the gpu
TORCH_DEVICE = torch.device("cpu")
# if torch.cuda.is_available():
#     TORCH_DEVICE = torch.device("cuda:0")
