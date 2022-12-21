from robot_brain.controller.push.mppi.mppi import PushMppi
import numpy as np
from robot_brain.state import State
import torch

# TODO: this is not pushging yet?
# TODO: is this second or a larger order system? name it correctly
class PushMppi5thOrder(PushMppi):
    """ Mppi push controller specific for 2th order systems
    it corresponds to the point robot which is driven by velocity input. """
    
    def __init__(self):
        PushMppi.__init__(self, order=5)

    def _running_cost(self, x: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        """ penalty function for input when the system is in a state, the running 
        cost drives the system to it's desired state. """
        return (x[:,2] - self.target_state.pos[0])**2 +\
                (x[:,3] - self.target_state.pos[1])**2 +\
                (x[:,4] - self.target_state.ang_p[2])**2 +\
                1e-4*(u[:,0]**4 + u[:,1]**4)

    def _find_input(self, robot_state: State, obstacle_state: State) -> np.ndarray:

        return self.controller.command(
                np.append(robot_state.get_xy_position(),
                obstacle_state.get_2d_pose(), axis=0)).cpu().numpy()
        
    def _simulate(self, robot_state: State, obstacle_state: State, system_input: np.ndarray) -> State: 
        """ simulate one time step into the future. """

        # TODO: simulate forward using both robot and obstacle state
        pose_2d = self.dyn_model(torch.reshape(
            torch.Tensor(np.append(robot_state.get_xy_position(),
                obstacle_state.get_2d_pose(), axis=0)), (1,5)),
            torch.reshape(torch.Tensor(system_input), (1,2))).numpy()[0]

        return State(pos=np.array([pose_2d[0], pose_2d[1], 0]), ang_p=np.array([0,0,pose_2d[2]]))

    def _calculate_prediction_error(self, robot_state: State, obstacle_state: State) -> float:
        """ return calculated prediction error. """

        # robot and obstacle state please
        return self.y_predicted.pose_euclidean(robot_state)
