from robot_brain.controller.mppi.mppi import Mppi
import numpy as np
from robot_brain.state import State
import torch

class Mppi4thOrder(Mppi):
    """ Mppi controller specific for 4th order systems
    it corresponds to the point robot which is driven by acceleration input. """
    
    def __init__(self):
        Mppi.__init__(self, order=4)

    def _running_cost(self, x: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        """ penalty function for input when the system is in a state, the running 
        cost drives the system to it's desired state. """
        return torch.subtract(x[:,0], self.target_state.pos[0])**2 +\
                torch.subtract(x[:,1], self.target_state.pos[1])**2 +\
                torch.subtract(x[:,2], self.target_state.vel[0])**2 +\
                torch.subtract(x[:,3], self.target_state.vel[1])**2 +\
                1e-4*(u[:,0]**4 + u[:,1]**4)

    def _find_input(self, current_state: State) -> np.ndarray:
        return self.controller.command(current_state.get_xy_dxdy()).cpu().numpy()
        
    def _simulate(self, current_state: State, system_input: np.ndarray) -> State: 
        """ simulate one time step into the future. """
        xy_dxdy = self.dyn_model(torch.reshape(torch.Tensor(current_state.get_xy_dxdy()), (1, 4)),
                    torch.reshape(torch.Tensor(system_input), (1,2))).numpy()[0]

        return State(pos=np.array([xy_dxdy[0], xy_dxdy[1], 0]),
                        vel=np.array([xy_dxdy[2], xy_dxdy[3], 0]))

    def _calculate_prediction_error(self, current_state: State) -> float:
        """ return calculated prediction error. """
        # todo: check that this eucludean is not fucking stuff up
        return self.y_predicted.euclidean(current_state)
