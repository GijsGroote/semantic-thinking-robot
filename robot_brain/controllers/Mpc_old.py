import numpy as np
import do_mpc
from casadi import *
import matplotlib as mpl
# mpl.use("TkAgg")
import matplotlib.pyplot as plt


class Mpc:
    def __init__(self):
        self._controller = None
        self._dyn_model = None
        self.sim_graphics = None
        self.simulator = None
        self.mpc_graphics = None
        self.fig = None
        self.dt = None


    def create_mpc_controller(self, dt, dyn_model, targetState, x0):
        self._dyn_model = dyn_model # todo: you are actually doing nothing with this model dude
        # todo: this should not be hardcoded
        self.dt = dt

        pos_x_t = targetState.pos[0]
        pos_y_t = targetState.pos[1]
        ang_p_t = 0.0

        # variables which can be tweaked
        setup_mpc = {
            'n_horizon': 10,
            't_step': self.dt,
            'n_robust': 1,
            'store_full_solution': False,
        }

        rterm_u1 = 1e-2
        rterm_u2 = 1e-2



        # setup model object
        model_type = 'discrete'
        model = do_mpc.model.Model(model_type)

        # state variables
        pos_x = model.set_variable(var_type='_x', var_name='pos_x', shape=(1, 1))
        pos_y = model.set_variable(var_type='_x', var_name='pos_y', shape=(1, 1))
        ang_p = model.set_variable(var_type='_x', var_name='ang_p', shape=(1, 1))

        # dx = model.set_variable(var_type='_x', var_name='x_kp1', shape=(3, 1))

        # inputs
        u1 = model.set_variable(var_type='_u', var_name='u1', shape=(1, 1))
        u2 = model.set_variable(var_type='_u', var_name='u2', shape=(1, 1))

        # # target inputs
        # u1_set = model.set_variable(var_type='_u', var_name='u1_set', shape=(1, 1))
        # u2_set = model.set_variable(var_type='_u', var_name='u2_set', shape=(1, 1))

        # pos_x_target = model.set_variable(var_type='_x', var_name='pos_x_target', shape=(1, 1))
        # pos_y_target = model.set_variable(var_type='_x', var_name='pos_y_target', shape=(1, 1))
        # ang_p_target = model.set_variable(var_type='_x', var_name='ang_p_target', shape=(1, 1))
        # model.set_rhs('pos_x_target', pos_x_t)
        # model.set_rhs('pos_y_target', pos_y_t)
        # model.set_rhs('ang_p_target', ang_p_t)

        # # true position
        # pos_x_true = model.set_variable(var_type='_x', var_name='pos_x_true', shape=(1, 1))
        # pos_y_true = model.set_variable(var_type='_x', var_name='pos_y_true', shape=(1, 1))
        # ang_p_true = model.set_variable(var_type='_x', var_name='ang_p_true', shape=(1, 1))


        # todo: set uncertain variables

        # set right hand site, todo: this should come from the dynamics model
        model.set_rhs('pos_x', pos_x)
        model.set_rhs('pos_y', pos_y)
        model.set_rhs('ang_p', ang_p)

        # right hand side equation f(x)
        model.set_rhs('pos_x', 0.1 * pos_x + np.cos(ang_p) * u1)
        model.set_rhs('pos_y', 0.1 * pos_y + np.sin(ang_p) * u1)
        model.set_rhs('ang_p', ang_p + 0.1 * u2)

        # this is still a bit vague is it not
        # tau = 0.01
        # model.set_rhs('pos_x_true', 1/tau*(3 - pos_x_true))
        # model.set_rhs('pos_y_true', 1/tau*(3 - pos_y_true))
        # model.set_rhs('ang_p_true', 1/tau*(1 - ang_p_true))

        model.setup()
        mpc = do_mpc.controller.MPC(model)

        # do not punish using input to much
        mpc.set_rterm(
            u1=rterm_u1,
            u2=rterm_u2
        )

        mpc.set_param(**setup_mpc)


        mterm = (pos_x) ** 2 + (pos_y) ** 2 + (ang_p) ** 2
        lterm = (pos_x) ** 2 + (pos_y) ** 2 + (ang_p) ** 2
        mpc.set_objective(mterm=mterm, lterm=lterm)

        # Lower bounds on states:
        # mpc.bounds['lower', '_x', 'ang_p'] = -2 * np.pi
        # Upper bounds on states
        # mpc.bounds['upper', '_x', 'ang_p'] = 2 * np.pi

        # Lower bounds on inputs:
        mpc.bounds['lower', '_u', 'u1'] = -5
        mpc.bounds['lower', '_u', 'u2'] = -5
        # upper bounds on inputs:
        mpc.bounds['upper', '_u', 'u1'] = 5
        mpc.bounds['upper', '_u', 'u2'] = 5

        mpc.setup()

        # simulating
        simulator = do_mpc.simulator.Simulator(model)
        simulator.set_param(t_step=self.dt)

        simulator.setup()

        simulator.x0 = x0
        self.simulator = simulator
        mpc.x0 = x0


        mpc.set_initial_guess()

        self.controller = mpc

        # Customizing Matplotlib:
        mpl.rcParams['font.size'] = 18
        mpl.rcParams['lines.linewidth'] = 3
        mpl.rcParams['axes.grid'] = True

        mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
        sim_graphics = do_mpc.graphics.Graphics(simulator.data)
        # We just want to create the plot and not show it right now. This "inline magic" supresses the output.
        fig, ax = plt.subplots(2, sharex=True, figsize=(16, 9))
        fig.align_ylabels()


        for g in [sim_graphics, mpc_graphics]:
            # Plot the angle positions (phi_1, phi_2, phi_2) on the first axis:
            g.add_line(var_type='_x', var_name='pos_x', axis=ax[0])
            g.add_line(var_type='_x', var_name='pos_y', axis=ax[0])
            g.add_line(var_type='_x', var_name='ang_p', axis=ax[0])

            # Plot the set motor positions (phi_m_1_set, phi_m_2_set) on the second axis:
            g.add_line(var_type='_u', var_name='u1', axis=ax[1])
            g.add_line(var_type='_u', var_name='u2', axis=ax[1])

        ax[0].set_ylabel('position [m]')
        ax[1].set_xlabel('time [s]')

        self.sim_graphics = sim_graphics
        self.mpc_graphics = mpc_graphics
        self.fig = fig

    def respond(self, x0):
        u0 = self.controller.make_step(x0)

        # todo: plot interative plot which does work
        # self.simulator.make_step(u0)
        #
        #
        # self.sim_graphics.plot_results()
        # # Reset the limits on all axes in graphic to show the data.
        # self.sim_graphics.reset_axes()
        # # Show the figure:
        # self.fig.show()



        return u0
