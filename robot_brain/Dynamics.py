from robot_brain.State import State


class Dynamics:
    def __init__(self):
        self.m = "test"

    def update(self, state, dt, force_x, force_y):
        # update dynamics for time step
        a_x = state.acc_x + force_x
        a_y = state.acc_y + force_y

        v_x = state.vel_x + dt*a_x
        v_y = state.vel_y + dt*a_y

        p_x = state.pos_x + 0.5*a_x*dt**2
        p_y = state.pos_y + 0.5*a_y*dt**2

    # mass getter
    @property
    def m(self):
        return self._m

    # mass setter
    @m.setter
    def m(self, value):
        if True: # TODO: input sanitization
            self._m = value

