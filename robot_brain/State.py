class State:
    def __init__(self, x, y):
        # position, velocity and acceleration in x and y direction
        self.pos_x = x
        self.vel_x = y
        self.acc_x = 0
        self.pos_y = 0
        self.vel_y = 0
        self.acc_y = 0

        # angular position, velocity and acceleration, currently not possible to rotate
        self.ang_p = 0
        self.ang_v = 0
        self.ang_a = 0

    # x position getter
    @property
    def pos_x(self):
        return self._pos_x

    # x position setter
    @pos_x.setter
    def pos_x(self, value):
        if True: # TODO: input sanitization
            self._pos_x = value

 # TODO: other getters and setters