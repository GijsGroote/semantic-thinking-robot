import numpy as np

class Dynamics:
    """
    Dynamical object.

    TODO: this class requires quite some review, how to handle different dynamical object
    single, multi object models and stuff.
    """
    def __init__(self):
        self.mass = "test"
        self._A = None
        self._B = None
        self._C = None
        self._D = None


    def set_boxer_model(self):
        self._A = np.array(([0.1, 0, 0], [0, 0.1, 0], [0, 0, 1]))
        self._B = np.array([1]) # fuck this is a nonlinear function

    # def update(self, state, dt, force_x, force_y):
    #     # update dynamics for time step
    #     a_x = state.acc_x + force_x
    #     a_y = state.acc_y + force_y
    #
    #     v_x = state.vel_x + dt*a_x
    #     v_y = state.vel_y + dt*a_y
    #
    #     p_x = state.pos_x + 0.5*a_x*dt**2
    #     p_y = state.pos_y + 0.5*a_y*dt**2


    # mass getter
    @property
    def mass(self):
        return self._mass

    # mass setter
    @mass.setter
    def mass(self, value):
        # if True: # TODO: input sanitization
        self._mass = value
