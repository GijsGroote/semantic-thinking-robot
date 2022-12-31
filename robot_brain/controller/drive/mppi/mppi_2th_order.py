from robot_brain.controller.drive.mppi.mppi import Mppi
from robot_brain.global_variables import TORCH_DEVICE
import numpy as np
from robot_brain.state import State
import torch

class DriveMppi2thOrder(Mppi):
    """ Mppi controller specific for 2th order systems
    it corresponds to the point robot which is driven by velocity input. """
    
    def __init__(self):
        Mppi.__init__(self, order=2)

    def _running_cost(self, x: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        """ penalty function for input when the system is in a state, the running 
        cost drives the system to it's desired state. """

        # force tensors to be stored on TORCH_DEVICE
        
        # x_t = torch.tensor(x, device=TORCH_DEVICE)
        # u_t = torch.tensor(u, device=TORCH_DEVICE)

        return (x[:,0] - torch.tensor([self.target_state.pos[0]], device=TORCH_DEVICE))**2 +\
               (x[:,1] - torch.tensor([self.target_state.pos[1]], device=TORCH_DEVICE))**2 +\
                1e-4*(u[:,0]**4 + u[:,1]**4)

    def _find_input(self, current_state: State) -> np.ndarray:

        ret =  self.controller.command(torch.tensor(current_state.get_xy_position(), device=TORCH_DEVICE))
        #
        # print('ret and type(ret)')
        # print(ret)
        # print(type(ret))

        return ret
        
    def _simulate(self, current_state: State, system_input: np.ndarray) -> State: 
        """ simulate one time step into the future. """
        xy_pos = self.dyn_model(torch.reshape(torch.tensor(current_state.get_xy_position(), device=TORCH_DEVICE), (1,2)),
                        torch.reshape(torch.tensor(system_input), (1,2))).numpy()[0]

        return State(pos=np.array([xy_pos[0], xy_pos[1], 0]))

    def _calculate_prediction_error(self, current_state: State) -> float:
        """ return calculated prediction error. """
        return self.y_predicted.pose_euclidean(current_state)
