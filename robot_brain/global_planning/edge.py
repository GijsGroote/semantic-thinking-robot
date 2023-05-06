import numpy as np
from abc import ABC, abstractmethod
from robot_brain.state import State
from robot_brain.system_model import SystemModel


EDGE_INITIALISED = "edge:initialised"
EDGE_COMPLETED = "edge:completed"
EDGE_EXECUTING = "edge:executing"
EDGE_FAILED = "edge:failed"

class Edge(ABC):
    """
    Edge or transition, describes the way/method of transitioning from
    one Node to another Node.
    """
    def __init__(self, iden: int, source: int, to: int, verb: str, controller):
        self.iden = iden
        self.source = source
        self.to = to
        self.verb = verb
        self.controller = controller
        self.system_model = SystemModel(None)

        self._path_estimation = None
        self._path = None

        self.path_pointer = 0
        self.alpha = None

    @abstractmethod
    def ready_for_execution(self) -> bool:
        """ checks if all parameters are set to execute this transition. """

    @abstractmethod
    def view_completed(self, current_state: State) -> bool:
        """ check if the view (smallest target, the controller tries to reach) in reached. """

    @abstractmethod
    def completed(self) -> bool:
        """ returns true if the edge is completed, otherwise false. """

    @abstractmethod
    def increment_current_view(self):
        """ updates toward the next current target from path. """

    @abstractmethod
    def get_current_view(self) -> State:
        """ returns the current target the controller tries to steer toward. """

    @abstractmethod
    def respond(self) -> np.ndarray:
        """ respond to the current state. """

    def create_log(self) -> dict:
        """ return a dictionary with metrics. """

    @abstractmethod
    def set_executing_status(self):
        """ Sets executing status. """

    @abstractmethod
    def set_completed_status(self):
        """ Sets completed status. """



    @abstractmethod
    def to_string(self):
        """ Creates readable format of an Edge. """

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
    def system_model(self):
        return self._system_model

    @system_model.setter
    def system_model(self, val):
        assert isinstance(val, SystemModel), f"system_model must be of type SystemModel and is {type(val)}"
        self._system_model = val

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
