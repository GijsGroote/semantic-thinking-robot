import do_mpc
import numpy as np
from robot_brain.controller.mpc.template_model import template_model
from robot_brain.controller.mpc.template_mpc import template_mpc
from robot_brain.controller.mpc.template_plotter import Plotter
from robot_brain.global_variables import *
from robot_brain.controller.Controller import Controller


class Mpc(Controller):

    def __init__(self):
        Controller.__init__(self)
        self.mpc = None
        self.model = None
        self.simulator = None
        self.plotter = None
        self.targetState = None
        self.n_horizon = 15


    def setup(self, dyn_model, currentState, targetState):
        """
            Setup MPC controller
            Specify dyn_model as a function as:

            def dyn_model(x, u):
                dx_next = vertcat(
                        x[0] + 0.05*np.cos(x[2]) * u[0],
                        x[1] + 0.05*np.sin(x[2]) * u[0],
                        x[2] + 0.05 * u[1])
                return dx_next
        """
        self.targetState = targetState

        # fully define model
        self.model = template_model(dyn_model)

        # set all mpc parameters
        self.mpc = template_mpc(self.model, self.n_horizon, targetState)
        
        x0 = np.array([currentState.pos[0], currentState.pos[0], currentState.ang_p[2]])
        self.mpc.x0 = x0

        self.mpc.set_initial_guess()

        
        if PLOT_CONTROLLER or CREATE_SERVER_DASHBOARD:

            simulator = do_mpc.simulator.Simulator(self.model)
            tvp_template = simulator.get_tvp_template()

            def tvp_fun(t_now):
                tvp_template['pos_x_target'] = targetState.pos[0] 
                tvp_template['pos_y_target'] = targetState.pos[1] 
                tvp_template['ang_p_target'] = targetState.ang_p[2]

                return tvp_template

            simulator.set_tvp_fun(tvp_fun)


            # Set parameter(s):
            simulator.set_param(t_step=DT)
            simulator.setup()

            # just to be sure
            simulator.reset_history()

            simulator.x0 = x0

            self.simulator = simulator
            self.plotter = Plotter()
            self.plotter.setup(self.mpc, self.simulator)

            self.y_predicted = currentState.get2DPose()
            self.predError = []

        self.mpc.reset_history()
            
        
    def findInput(self, currentState):
        x0 = currentState.get2DPose()
        self.mpc.x0 = x0 
        u0 = self.mpc.make_step(x0) # solves minimisation problem 

        if PLOT_CONTROLLER or CREATE_SERVER_DASHBOARD: 
            # calculate one step ahead prediction error
            self.predError.append(np.linalg.norm(self.y_predicted - x0))

            self.simulator.x0 = x0
            self.y_predicted = np.reshape(self.simulator.make_step(u0), x0.shape)

        return u0
    def setTargetState(self, state):

        self.targetState = state

        tvp_template = self.mpc.get_tvp_template()

        def tvp_fun(t_now):
            for k in range(self.n_horizon+1):
                tvp_template['_tvp',k,'pos_x_target'] = state.pos[0] 
                tvp_template['_tvp',k,'pos_y_target'] = state.pos[1] 
                tvp_template['_tvp',k,'ang_p_target'] = state.ang_p[2]

            return tvp_template

        self.mpc.set_tvp_fun(tvp_fun)


    def visualise(self):
        self.plotter.visualise(self.targetState, self.predError)
        
    def updateDB(self):
        self.plotter.updateDB(self.targetState, self.predError)






