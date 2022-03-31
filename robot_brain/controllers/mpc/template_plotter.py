import do_mpc
import pandas as pd

pd.options.plotting.backend = "plotly"
from plotly.subplots import make_subplots
import plotly.express as px
import numpy as np
import plotly.graph_objects as go
import os
import pyarrow.feather as feather



class Plotter():

    def __init__(self):
        self.mpc_graphics = None
        self.sim_graphics = None
        self.simulator = None
        self.mpc = None

    def setup(self, mpc, simulator):
        self.mpc = mpc
        self.simulator = simulator

        # note: this as mpc_graphics and another sim_graphics
        # graphics = do_mpc.graphics.Graphics(mpc.data)
        self.mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
        self.sim_graphics = do_mpc.graphics.Graphics(simulator.data)

    def update(self, dt, current_time):
        """
        Stores the MPC data as feather file.
        """

        dictionary = {
            "type": "mpc",
            "x": [i[0] for i in self.simulator.data["_x"]],
            "y": [i[1] for i in self.simulator.data["_x"]],
            "theta": [i[2] for i in self.simulator.data["_x"]],
            "u1": [i[0] for i in self.simulator.data['_u']],
            "u2": [i[1] for i in self.simulator.data['_u']]
        }

        # todo: metadata so I can tell this data is mpc data
        df = pd.DataFrame(dictionary)

        # only store data which will be plot
        if current_time >= 15:
            time = np.arange(current_time-15, current_time, dt)
            df = df.tail(len(time))
            df["time"] = time
        else:
            df["time"] = np.arange(df.index[0], current_time, dt)

        feather.write_feather(df, '../robot_brain/dashboard/data/mpc_data.feather')
