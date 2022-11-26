import numpy as np
from abc import ABC, abstractmethod
from robot_brain.state import State

class Edge(ABC):
    """
    Edge or transition, describes the way/method of transitioning from
    one Node to another Node.
    """
    def __init__(self, iden, source, to, verb, controller):
        self.iden = iden
        self.source = source
        self.to = to
        self.verb = verb
        self.controller = controller
        self._dyn_model = None 

        self._path_estimation = None
        self._path = None

        self.path_pointer = 0
        self.alpha = None

    @abstractmethod
    def ready_for_execution(self) -> bool:
        """ checks if all parameters are set to execute this transition. """
        pass
    
    @abstractmethod
    def view_completed(self) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """
        pass

    @abstractmethod
    def completed(self) -> bool:
        """ returns true if the edge is completed, otherwise false. """
        pass

    @abstractmethod
    def increment_current_target(self):
        """ updates toward the next current target from path. """
        pass

    @abstractmethod
    def get_current_target(self) -> State:
        """ returns the current target the controller tries to steer toward. """
        pass

    @abstractmethod
    def respond(self, state) -> np.ndarray:
        """ respond to the current state. """
        pass

    def create_log(self) -> dict:
        """ return a dictionary with metrics. """
        pass

    @abstractmethod
    def to_string(self):
        """ Creates readable format of an Edge. """
        pass


    @property
    def iden(self):
        return self._iden

    @iden.setter
    def iden(self, val):
        assert isinstance(val, int), f"iden should be of type int and is of type {type(val)}"
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
    def dyn_model(self):
        return self._dyn_model

    @dyn_model.setter
    def dyn_model(self, func):
        assert callable(func), f"dyn_model must be a callable function and is {type(func)}"
        self._dyn_model = func

    @property
    def path_estimation(self):
        return self._path_estimation

    @path_estimation.setter
    def path_estimation(self, path_estimation):
        # TODO: check plan is a plan
        self._path_estimation = path_estimation

    @property
    def path(self):
        return self._path

    @path.setter
    def path(self, path):
        # TODO: check plan is a plan
        self._path = path
