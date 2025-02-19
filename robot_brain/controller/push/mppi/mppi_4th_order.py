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
        self.name = "MPPI_4th_order"

    def _running_cost(self, x: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        """ penalty function for input when the system is in a state, the running
        cost drives the system to it's desired state. """

        # target vector in torch format
        target_points = torch.cat((self.target_state.pos[0]*torch.ones((x.size(dim=0), 1), device=TORCH_DEVICE),
                self.target_state.pos[1]*torch.ones((x.size(dim=0), 1), device=TORCH_DEVICE)), 1)

        # point a: object target
        # point b: the object
        # point c: The robot

        # penalty for object not on the target position
        obj_to_target_cost = torch.norm((target_points - x[:,2:4]), p=2, dim=1)

        # Calculate vector from a to b
        ab = target_points - x[:,0:2]
        ac = target_points - x[:,2:4]

        # project ac onto ab
        # proj_ac_on_ab = torch.sum(ab * ac, dim=1, keepdim=True)/torch.sum(ac * ac, dim=1, keepdim=True)*ab
        # print(f'projection of ac onto ab {proj_ac_on_ab[0,:]}')
        # closest = ac-proj_ac_on_ab
        # print(f'distance rom that lineof ac onto ab {closest[0,:]}')
        # Calculate distance point c to line segment ab

        norm_c = torch.norm(target_points, dim=1, keepdim=True)
        unit_c = target_points / norm_c
        dot = torch.sum((ab-ac) * unit_c, dim=1, keepdim=True)
        # projection = dot * unit_c

        closest = x[:,2:4] + torch.clamp(dot, max=0) * unit_c

        # Calculate distance between a and closest point
        robot_behind_obj_cost = torch.norm(x[:,0:2] - closest, dim=1)
# print(f"object at {x[0, 2:4]} robot at {x[0,0:2]} target at {self.target_state.pos[0:2]}")
        return robot_behind_obj_cost# + obj_to_target_cost
        # return robot_behind_obj_cost

    def _find_input(self, robot_state: State, obstacle_state: State) -> np.ndarray:

        return self.controller.command(
                np.append(robot_state.get_xy_position(),
                obstacle_state.get_xy_position(), axis=0)).cpu().numpy()

    def _simulate(self, robot_state: State, obstacle_state: State, system_input: np.ndarray) -> State:
        """ simulate one time step into the future. """

        pose_2d = self.system_model.model(torch.reshape(
            torch.Tensor(np.append(robot_state.get_xy_position(),
                obstacle_state.get_2d_pose(), axis=0)), (1,5)),
            torch.reshape(torch.Tensor(system_input), (1,2))).numpy()[0]

        return State(pos=np.array([pose_2d[0], pose_2d[1], 0]), ang_p=np.array([0,0,pose_2d[2]]))

    def _calculate_prediction_error(self, obst_state: State) -> float:
        """ return calculated prediction error. """
        return self.y_predicted.position_euclidean(obst_state)
