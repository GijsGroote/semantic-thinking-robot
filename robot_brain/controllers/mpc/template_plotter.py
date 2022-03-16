import do_mpc
import matplotlib as mpl
import numpy as np
# mpl.use("TkAgg")
import matplotlib.pyplot as plt
import mpld3



class Plotter():

    def __init__(self):
        self.fig = None
        self.mpc_graphics = None
        self.sim_graphics = None


    def setup(self, mpc, simulator):

        # plotting
        # Customizing Matplotlib:
        plt.ion()
        mpl.rcParams['font.size'] = 18
        mpl.rcParams['lines.linewidth'] = 3
        mpl.rcParams['axes.grid'] = True

        # note: this as mpc_graphics and another sim_graphics
        # graphics = do_mpc.graphics.Graphics(mpc.data)
        self.mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
        self.sim_graphics = do_mpc.graphics.Graphics(simulator.data)

        self.fig, ax = plt.subplots(2, figsize=(10, 6))

        self.fig.align_ylabels()

        self.sim_graphics.add_line(var_type='_x', var_name='pos_x', axis=ax[0], label='x')
        self.sim_graphics.add_line(var_type='_x', var_name='pos_y', axis=ax[0], label='y')
        self.sim_graphics.add_line(var_type='_x', var_name='ang_p', axis=ax[0], label='theta')

        # Plot the set motor positions (phi_m_1_set, phi_m_2_set) on the second axis:
        self.sim_graphics.add_line(var_type='_u', var_name='u1', axis=ax[1], label='u1')
        self.sim_graphics.add_line(var_type='_u', var_name='u2', axis=ax[1], label='u2')

        # for g in [sim_graphics, mpc_graphics]:
        #     # Plot the angle positions (phi_1, phi_2, phi_2) on the first axis:
        #     g.add_line(var_type='_x', var_name='pos_x', axis=ax[0])
        #     g.add_line(var_type='_x', var_name='pos_y', axis=ax[0])
        #     g.add_line(var_type='_x', var_name='ang_p', axis=ax[0])
        #
        #     # Plot the set motor positions (phi_m_1_set, phi_m_2_set) on the second axis:
        #     g.add_line(var_type='_u', var_name='u1', axis=ax[1])
        #     g.add_line(var_type='_u', var_name='u2', axis=ax[1])

        # todo: position in meter?, input in? really
        ax[0].legend(loc='upper left')
        ax[1].legend(loc='lower left')
        ax[0].set_ylabel('position [m]')
        ax[1].set_ylabel('input')
        ax[1].set_xlabel('time [sec]')

        # plt.draw()
        plt.draw()

    def update(self):
        self.sim_graphics.plot_results()
        self.sim_graphics.reset_axes()

        # self.mpc_graphics.plot_results()
        # self.mpc_graphics.reset_axes()

        # Reset the limits on all axes in graphic to show the data.

        plt.draw()
        plt.pause(1e-9)


        html_str = mpld3.fig_to_html(self.fig)
        Html_file = open("../robot_brain/dashboard/mpc.html", "w")
        Html_file.write(html_str)
        Html_file.close()




