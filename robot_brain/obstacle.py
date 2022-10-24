from motion_planning_env.free_collision_obstacle import FreeCollisionObstacle
from motion_planning_env.box_obstacle import BoxObstacle


class Obstacle:
    """
    Obstacle class.
    """
    def __init__(self, name, state, properties):
        self.name = name
        self.state = state
        # create empty nonetype properties
        if properties == "empty":
            box_dict = {
                "movable": False,
                "type": "box",
                "color": [0/255, 255/255, 0/255, 1],
                "position": [0, 0, 0],
                "geometry": {"length": 1, "width": 1, "height": 1},
            }
            properties = BoxObstacle(name="None-Type-Obstacle", content_dict=box_dict) 

        self.properties = properties
        self.type = "unknown"

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
    def properties(self) -> FreeCollisionObstacle:
        return self._properties

    # property setter
    @properties.setter
    def properties(self, val: FreeCollisionObstacle):
        self._properties = val

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
