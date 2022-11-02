from robot_brain.controller.mppi.mppi import Mppi
import numpy as np
from robot_brain.state import State
import torch

class Mppi6thOrder(Mppi):
    """ Mppi controller specific for 6th order systems
    it corresponds to the boxer robot which is driven by acceleration input. """
    
    def __init__(self):
        Mppi.__init__(self, order=6)

    def _running_cost(self, x: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        """ penalty function for input when the system is in a state, the running 
        cost drives the system to it's desired state. """
        return torch.subtract(x[:,0], self.target_state.pos[0])**2 +\
                torch.subtract(x[:,1], self.target_state.pos[1])**2 +\
                torch.subtract(x[:,2], self.target_state.ang_p[2])**2 +\
                torch.subtract(x[:,3], self.target_state.vel[0])**2 +\
                torch.subtract(x[:,4], self.target_state.vel[1])**2 +\
                torch.subtract(x[:,5], self.target_state.ang_v[2])**2 +\
                1e-4*(u[:,0]**4 + u[:,1]**4)

    def _find_input(self, current_state: State) -> np.ndarray:
        return self.controller.command(current_state.get_xyt_dxdydt()).cpu().numpy()
        
    def _simulate(self, current_state: State, system_input: np.ndarray) -> State: 
        """ simulate one time step into the future. """
        xyt_dxdydt = self.dyn_model(torch.reshape(torch.Tensor(current_state.get_xyt_dxdydt()), (1,6)),
                    torch.reshape(torch.Tensor(system_input), (1,2))).numpy()[0]

        return State(pos=np.array([xyt_dxdydt[0], xyt_dxdydt[1], 0]),
                     ang_p=np.array([0, 0, xyt_dxdydt[2]]),
                     vel=np.array([xyt_dxdydt[3], xyt_dxdydt[4], 0]),
                     ang_v=np.array([0, 0, xyt_dxdydt[5]]))

    def _calculate_prediction_error(self, current_state: State) -> float:
        """ return calculated prediction error. """
        # todo: check that this eucludean is not fucking stuff up
        return self.y_predicted.euclidean(current_state)
