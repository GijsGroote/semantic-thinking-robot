import numpy as np


class Configuration:

    # todo: this entire class

    """
    the configuration can be created with the state and some booleans, or just with everything in the configuration yo

    """

    def __init__(self):
        def __init__(self, pos=np.array([0, 0, 0]), vel=np.array([0, 0, 0]), acc=np.array([0, 0, 0]),
                     ang_p=np.array([0, 0, 0]), ang_v=np.array([0, 0, 0]), ang_a=np.array([0, 0, 0])):
            # position, velocity and acceleration in x and y direction
            self.pos = pos
            self.vel = vel
            self.acc = acc
            self.ang_p = ang_p
            self.ang_v = ang_v
            self.ang_a = ang_a
