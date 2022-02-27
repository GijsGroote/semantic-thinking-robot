import do_mpc
import numpy as np
from robot_brain.controllers.mpc.template_model import template_model
from robot_brain.controllers.mpc.template_mpc import template_mpc
from robot_brain.controllers.mpc.template_simulator import template_simulator
from robot_brain.controllers.mpc.template_estimator import template_estimator
import matplotlib as mpl
import matplotlib.pyplot as plt

class Mpc:
    def __init__(self):
        self.mpc = None
        self.dyn_model = None
        self.model = None
        self.mpc_graphics = None
        self.sim_graphics = None
        self.simulator = None
        self.y_next = np.array([0, 0, 0])
        self.fig = None


    def create_mpc_controller(self, dt, dyn_model, currentState, targetState):

        self.model = template_model()
        self.mpc = template_mpc(self.model, targetState, dt)
        self.simulator = template_simulator(self.model, dt)
        # template estiamtor needs some more work yo
        # mhe = template_estimator(model, dt)

        #  for now this please
        estimator = do_mpc.estimator.StateFeedback(self.model)
        x0 = np.array([currentState.pos[0], currentState.pos[0], currentState.ang_p[2]])
        self.simulator.x0 = x0
        self.mpc.x0 = x0

        self.mpc.set_initial_guess()

        # plotting
        # Customizing Matplotlib:
        mpl.rcParams['font.size'] = 18
        mpl.rcParams['lines.linewidth'] = 3
        mpl.rcParams['axes.grid'] = True

        # note: this as mpc_graphics and another sim_graphics
        # graphics = do_mpc.graphics.Graphics(mpc.data)
        self.mpc_graphics = do_mpc.graphics.Graphics(self.mpc.data)
        self.sim_graphics = do_mpc.graphics.Graphics(self.simulator.data)

        self.fig, ax = plt.subplots(2, sharex=True, figsize=(16, 9))
        self.fig.align_ylabels()

        for g in [self.sim_graphics, self.mpc_graphics]:
            # Plot the angle positions (phi_1, phi_2, phi_2) on the first axis:
            g.add_line(var_type='_x', var_name='pos_x', axis=ax[0])
            g.add_line(var_type='_x', var_name='pos_y', axis=ax[0])
            g.add_line(var_type='_x', var_name='ang_p', axis=ax[0])

            # Plot the set motor positions (phi_m_1_set, phi_m_2_set) on the second axis:
            g.add_line(var_type='_u', var_name='u1', axis=ax[1])
            g.add_line(var_type='_u', var_name='u2', axis=ax[1])

        ax[0].set_ylabel('position [m]')
        ax[1].set_xlabel('time [s]')

    def plot(self):
        self.sim_graphics.plot_results()

        # Reset the limits on all axes in graphic to show the data.
        self.sim_graphics.reset_axes()

        # graphics.reset_axes()
        # graphics.plot_results()
        # graphics.plot_predictions()

        # Show the figure:
        self.fig.show()



    def respond(self, currentState):
        x0 = np.array([currentState.pos[0], currentState.pos[1], currentState.ang_p[2]])
        u0 = self.mpc.make_step(x0)

        self.y_next = np.reshape(self.simulator.make_step(u0), (3,))





        # self.plot()

        return u0


