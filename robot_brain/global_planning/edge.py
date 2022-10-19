from robot_brain.state import State
import numpy as np

class Edge:
    """
    Edge or transition, describes the way/method of transitioning from
    one Node to another Node.
    """
    def __init__(self, iden, source, to, verb, controller, path=False):
        self.iden = iden
        self.source = source
        self.to = to
        self.verb = verb
        self.controller = controller
        self.dyn_model = "use a al dynamical model please"
        self.path = path
        self.path_pointer = 0
        self.alpha = None

    def to_string(self):
        """
        Creates readable format of an Edge
        :return: String
        """
        return f"id: {self.iden}\nverb: {self.verb}"
    
    def completed(self) -> bool:
        """ returns true if the path is completed, otherwise false. """
        return self.path_pointer >= len(self.path)-1
    
    def increment_current_target(self):
        """ updates toward the next current target from path. """

        if self.path_pointer < len(self.path)-1:
            self.path_pointer += 1

        if  len(self.path[self.path_pointer]) == 3:
            orien = self.path[self.path_pointer][2]
        else:
            orien = 0

        next_target = State(
                pos=np.array([self.path[self.path_pointer][0], self.path[self.path_pointer][1], 0]),
                ang_p=np.array([0, 0, orien])
                )

        self.controller.set_target_state(next_target)

        print(f"target reached, now setting {self.path[self.path_pointer]} as goal")

    def get_current_target(self) -> State:
        """ returns the current target the controller tries to steer toward. """
        if len(self.path[self.path_pointer]) == 3:
            orien = self.path[self.path_pointer][2]
        else:
            orien = 0
        return State(pos=np.append(self.path[self.path_pointer][0:2], [[0]]),
                ang_p=np.array([0, 0, orien]))

    def respond(self, state) -> np.ndarray:
        """ respond to the current state. """
        return self.controller.respond(state)

    @property
    def iden(self):
        return self._iden

    @iden.setter
    def iden(self, val):
        self._iden = val

    @property
    def source(self):
        return self._source

    @source.setter
    def source(self, val):
        self._source = val

    @property
    def to(self):
        return self._to

    @to.setter
    def to(self, val):
        self._to = val

    @property
    def verb(self):
        return self._verb

    @verb.setter
    def verb(self, val):
        assert isinstance(val, str), f"verb should be of type str and is {type(val)}"
        self._verb = val

    @property
    def controller(self):
        return self._controller

    @controller.setter
    def controller(self, contr):
        # TODO: check contr is a Controller object
        self._controller = contr

    @property
    def path(self):
        return self._path

    @path.setter
    def path(self, path):
        # TODO: check plan is a plan
        self._path = path
