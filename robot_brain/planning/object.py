from motion_planning_env.free_collision_obstacle import FreeCollisionObstacle


class Object:
    """
    Object class.
    """
    def __init__(self, name, state, urfd):
        self.name = name
        self.obstacle = None
        self.type = "unknown"
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

    # obstacle getter
    @property
    def obstacle(self) -> FreeCollisionObstacle:
        return self._obstacle

    # obstacle setter
    @obstacle.setter
    def obstacle(self, val: FreeCollisionObstacle):
        self._obstacle = val

    # type getter
    @property
    def type(self):
        return self._type

    # type setter
    @type.setter
    def type(self, val):
        if val in {"unmovable", "movable", "unknown"}:
            self._type = val
        else:
            raise ValueError(f"the type {val} is not allowed")

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
