DT = 0.02
# dashboard global variables
CREATE_SERVER_DASHBOARD = True
PLOT_CONTROLLER = False
LOG_METRICS = True

# plot_controller should be True for SAVE_LOG_METRICS
SAVE_LOG_METRICS = False

# you could add a rotation constraints here as well
MIN_INPUT = -2.0
MAX_INPUT = 2.0


# the number of time steps to plot for any controller
PLOT_N_TIMESTEPS = 200 

FIG_BG_COLOR = "rgb(230, 230, 255)"  # myEvenLighterColor "rgba(229,236,246,255)"

# "rgb(178, 178, 255)" # #b2b2ff myDarkColor
# "rgb(204, 204, 255)" # #ccccff myLightColor
# "rgb(230, 230, 255)" # #e6e6ff myEvenLighterColor


# TODO: set this path automatically
PROJECT_PATH = "/home/gijs/Documents/semantic-thinking-robot/"

UNKNOWN_OBSTACLE_COST = 150
KNOWN_OBSTACLE_COST = 100

# MPC_CONTROLLER = "mpc controller"
