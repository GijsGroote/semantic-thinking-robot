import numpy as np

class Configuration:
    """
    the configuration can be created with the state and some booleans,
    or just with everything in the configuration yo
    """

    def __init__(self, pos=None, vel=None, acc=None, ang_p=None, ang_v=None, ang_a=None):

        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.ang_p = ang_p
        self.ang_v = ang_v
        self.ang_a = ang_a


    # TODO: setters and getters
