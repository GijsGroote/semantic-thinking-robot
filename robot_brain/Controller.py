import numpy as np
import do_mpc
from casadi import *
import matplotlib.pyplot as plt
import matplotlib as mpl


class Controller:
    """ OLD CLASS, USE THE CONTROLLER DIRECTORY """
    # todo: the controller object which has some hierarchy for controllers and must have some functions such as respond
    def __init__(self, dt):
        self._controller = None
        self._dyn_model = None
        self.sim_graphics = None
        self.simulator = None
        self.mpc_graphics = None
        self.fig = None
        self.dt = dt



