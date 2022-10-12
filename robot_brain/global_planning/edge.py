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
        self.temp_target = State(pos=np.array(path[0][0:2]), ang_p=np.array([0, 0, path[0][2]]))
        self.alpha = None

    def to_string(self):
        """
        Creates readable format of an Edge
        :return: String
        """
        return f"id: {self.iden}\nverb: {self.verb}"

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
