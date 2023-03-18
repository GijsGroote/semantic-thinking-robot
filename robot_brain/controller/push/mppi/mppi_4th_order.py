import torch
import numpy as np
from robot_brain.controller.push.mppi.mppi import PushMppi
from robot_brain.global_variables import TORCH_DEVICE
from robot_brain.state import State

class PushMppi4thOrder(PushMppi):
    """ Mppi push controller specific for 2th order systems
    it corresponds to the point robot which is driven by velocity input. """

    def __init__(self):
        PushMppi.__init__(self, order=4)

    def _running_cost(self, x: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        """ penalty function for input when the system is in a state, the running
        cost drives the system to it's desired state. """

        # target vector in torch format
        target_torch = torch.cat((torch.tensor([self.target_state.pos[0]], device=TORCH_DEVICE),
            torch.tensor([self.target_state.pos[1]], device=TORCH_DEVICE)), 0)

        # penalty for object not on the target position
        obj_to_target_cost = torch.norm((target_torch - x[:,2:4]), p=2, dim=1)

        # penalty for the robot not being in line with target position an objects current position
        b = x[:,2:4]-target_torch
        # a = x[:,0:2]
        # a1 = torch.div(a*b, b*b)*b
        # a2 = a - a1
        robot_to_line_cost = torch.norm(x[:,0:2]-torch.div(x[:,0:2]*b, b*b)*b, p=2, dim=1)

        # return obj_to_target_cost
        return obj_to_target_cost + robot_to_line_cost


    def _find_input(self, robot_state: State, obstacle_state: State) -> np.ndarray:

        return self.controller.command(
                np.append(robot_state.get_xy_position(),
                obstacle_state.get_xy_position(), axis=0)).cpu().numpy()

    def _simulate(self, robot_state: State, obstacle_state: State, system_input: np.ndarray) -> State:
        """ simulate one time step into the future. """

        # TODO: simulate forward using both robot and obstacle state
        pose_2d = self.system_model.model(torch.reshape(
            torch.Tensor(np.append(robot_state.get_xy_position(),
                obstacle_state.get_2d_pose(), axis=0)), (1,5)),
            torch.reshape(torch.Tensor(system_input), (1,2))).numpy()[0]

        return State(pos=np.array([pose_2d[0], pose_2d[1], 0]), ang_p=np.array([0,0,pose_2d[2]]))

    def _calculate_prediction_error(self, obst_state: State) -> float:
        """ return calculated prediction error. """
        return self.y_predicted.pose_euclidean(obst_state)
