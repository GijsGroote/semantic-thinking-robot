class Object:
    """
    Object class.
    """
    def __init__(self, name, state, urfd):
        self.name = name
        self.state = state
        self.urdf = urfd

    # name getter
    @property
    def name(self):
        return self._name

    # name setter
    @name.setter
    def name(self, val):
        self._name = val

    # state getter
    @property
    def state(self):
        return self._state

    # state setter
    @state.setter
    def state(self, stat):
        # TODO: input sanitization
        self._state = stat

    @property
    def urdf(self):
        return self._urdf

    # urdf setter
    @urdf.setter
    def urdf(self, val):
        # TODO: input sanitization
        self._urdf = val
