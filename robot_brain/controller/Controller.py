import warnings
from abc import ABC, abstractmethod
from robot_brain.global_variables import *

class Controller(ABC):

    def __init__(self):
        self.model = None
        self.path = None
        self.dt_counter = 0 
        
    @abstractmethod
    def setup(self, model, currentState, targetState):
        pass

    @abstractmethod
    def findInput(self, currentState):
        pass

    def respond(self, currentState):
        # update plotting data every second
        if CREATE_SERVER_DASHBOARD and PLOT_CONTROLLER:
            if self.dt_counter % (1/DT) == 0 and self.dt_counter != 0:
                self.updateDB()
            self.dt_counter += 1

        return self.findInput(currentState)


    
    @abstractmethod
    def visualise(self):
        pass

    @abstractmethod
    # updates the data for the DashBoard
    def updateDB(self):
        pass


