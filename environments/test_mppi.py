import numpy as np
import torch
from pytorch_mppi import mppi
from robot_brain.state import State

import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
import urdfenvs.boxer_robot # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.state import State
from robot_brain.global_variables import DT
from robot_brain.controller.mppi.mppi import Mppi


def main():
    """ MPPI test for the gym environment. """

    robot_type = "pointRobot-vel-v7"
    # robot_type = "pointRobot-acc-v7"
    # robot_type = "boxerRobot-vel-v7"
    # robot_type = "boxerRobot-acc-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())

    pos0 = np.array([1.0, 1.0])
    vel0 = np.array([0.0, 0.0])
    ob = env.reset(pos=pos0, vel=vel0)

    n_steps = 10000

    #
    # d = torch.device("cpu")
    #
    # def dynamics(x, u):
    #
    #     x_next = torch.zeros(x.shape, dtype=torch.float64, device=d)
    #
    #     x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
    #     x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]
    #
    #     return x_next
    #
    # targetState = State(pos=np.array([1,1,0]), ang_p=np.array([0,0,0]))
    # 
    # def running_cost(x, u):
    #     """ running_cost is euclidean distance toward target. """
    #     return torch.subtract(x[:,0], targetState.pos[0])**2 +\
    #             torch.subtract(x[:,1], targetState.pos[1])**2 +\
    #             1e-4*(u[:,0]**2 + u[:,0]**2)
    # 
    # def set_target_state(target_state):
    #     # new running cost function
    #     def running_cost(x, u):
    #         return torch.subtract(x[:,0], target_state.pos[0])**2 +\
    #                 torch.subtract(x[:,1], target_state.pos[1])**2 +\
    #                 1e-130*(u[:,0]**4 + u[:,1]**4)
    #     # set the new running cost
    #     ctrl.running_cost = running_cost
    #
    # # create controller with chosen parameters
    # ctrl = mppi.MPPI(dynamics=dynamics,
    #         running_cost=running_cost,
    #         nx=2,
    #         noise_sigma=torch.tensor([[1,0],[0,1]], device=d, dtype=torch.double),
    #         num_samples=1000, # number of rolouts
    #         horizon=5,
    #         lambda_=1e-2,
    #         # device=d, 
    #         u_min=torch.tensor([-2, -2], dtype=torch.double, device=d),
    #         u_max=torch.tensor([2, 2], dtype=torch.double, device=d)
    #         )

    def dynamics(x, u):

        x_next = torch.zeros(x.shape, dtype=torch.float64, device=torch.device("cpu"))

        x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
        x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]

        return x_next

    mppi_controller = Mppi(order=2)

    mppi_controller.setup(dyn_model=dynamics,
           current_state=State(),
           target_state=State(pos=np.array([1,1,0])))

    # add sensors
    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)


    # assuming you have a gym-like env
    for i in range(n_steps):

        if i == 50:
            mppi_controller.visualise(save=False)

        current_state = State(pos=ob["joint_state"]["position"])
        action[0:2] = mppi_controller.respond(current_state=current_state)

        # print(f'action: {action}')

        ob, reward, done, _ = env.step(action)
        # print(f"Observation: {ob}")


if __name__ == "__main__":
    main()
