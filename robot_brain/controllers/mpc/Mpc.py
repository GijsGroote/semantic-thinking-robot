import do_mpc
import numpy as np
from robot_brain.controllers.mpc.template_model import template_model
from robot_brain.controllers.mpc.template_mpc import template_mpc
from robot_brain.controllers.mpc.template_simulator import template_simulator
from robot_brain.controllers.mpc.template_estimator import template_estimator
from robot_brain.controllers.mpc.template_plotter import Plotter
from robot_brain.global_variables import *

class Mpc:
    def __init__(self):
        self.mpc = None
        self.dyn_model = None
        self.model = None
        self.mpc_graphics = None
        self.sim_graphics = None
        self.simulator = None
        self.plotter = None
        self.y_next = np.array([0, 0, 0])
        self.plot_results = None
        self.dt = None
        self.dt_counter = None



    def create_mpc_controller(self, dt, dyn_model, currentState, targetState):

        self.plot_results = PLOT_CONTROLLER

        self.model = template_model()
        self.mpc = template_mpc(self.model, targetState, dt, self.plot_results)
        self.simulator = template_simulator(self.model, dt)

        self.dt = dt

        x0 = np.array([currentState.pos[0], currentState.pos[0], currentState.ang_p[2]])
        self.mpc.x0 = x0
        self.simulator.x0 = x0

        self.mpc.set_initial_guess()


        if self.plot_results:
            self.dt_counter = 0
            self.plotter = Plotter()
            self.plotter.setup(self.mpc, self.simulator)

    def respond(self, currentState):
        x0 = np.array([currentState.pos[0], currentState.pos[1], currentState.ang_p[2]])
        u0 = self.mpc.make_step(x0)

        self.y_next = np.reshape(self.simulator.make_step(u0), (3,))

        if self.plot_results:

            self.dt_counter = self.dt_counter + 1

            if self.dt_counter % (1/self.dt) == 0:
                self.plotter.update(self.dt, self.dt_counter*self.dt)



        return u0


