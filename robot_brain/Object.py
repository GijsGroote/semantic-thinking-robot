class Object:
    def __init__(self, state, width=0, height=0, dynamics=None):
        self._state = state
        self._width = width
        self._height = height
        self._dynamics = dynamics

    # state getter
    @property
    def state(self):
        return self._state

    # state setter
    @state.setter
    def state(self, stat):
        if True:  # TODO: input sanitization
            self._state = stat

    @property
    def width(self):
        return self._width

    # width setter
    @width.setter
    def width(self, value):
        if True:  # TODO: input sanitization
            self._width = value

    # height getter
    @property
    def height(self):
        return self._height

    # height setter
    @height.setter
    def height(self, value):
        if True:  # TODO: input sanitization
            self._height = value

    # dynamics getter
    @property
    def dynamics(self):
        return self._dynamics

    # dynamics setter
    @dynamics.setter
    def dynamics(self, dyn):
        if True:  # TODO: input sanitization
            self._dynamics = dyn
