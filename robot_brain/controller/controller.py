from abc import ABC, abstractmethod
from robot_brain.global_variables import PLOT_CONTROLLER, CREATE_SERVER_DASHBOARD, DT

class Controller(ABC):
    """
    Controller class provides various single- and multi-body
    controllers, which calculate input for the robot.
    """

    def __init__(self):
        self.model = None
        self.path = None
        self.dt_counter = 0

    @abstractmethod
    def setup(self, model, current_state, target_state):
        pass

    @abstractmethod
    def find_input(self, current_state):
        pass

    def respond(self, current_state):
        # update plotting data every second
        if CREATE_SERVER_DASHBOARD and PLOT_CONTROLLER:
            if self.dt_counter % (1/DT) == 0 and self.dt_counter != 0:
                self.update_db()
            self.dt_counter += 1

        return self.find_input(current_state)

    @abstractmethod
    def visualise(self):
        pass

    @abstractmethod
    def update_db(self):
        pass
