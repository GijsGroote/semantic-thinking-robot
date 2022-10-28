import numpy as np
import torch
from pytorch_mppi import mppi
from casadi import vertcat
from robot_brain.state import State

import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
import urdfenvs.boxer_robot # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.state import State
from robot_brain.global_variables import DT


def main():
    """ MPPI test for the gym environment. """

    robot_type = "pointRobot-vel-v7"
    # robot_type = "pointRobot-acc-v7"
    # robot_type = "boxerRobot-vel-v7"
    # robot_type = "boxerRobot-acc-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())

    pos0 = np.array([1.0, 0.1])
    vel0 = np.array([0.0, 0.0])
    ob = env.reset(pos=pos0, vel=vel0)

    n_steps = 10000

    nx = 3
    nu = 2
    # network output is state residual
    H_UNITS = 32
    # ACTION_HIGH = 1
    # ACTION_LOW = -1

    d = torch.device("cpu")

    def dynamics(x, u):
        # print(f' u {u.shape}')
        # print(f' x {x.shape}')

        # x = x.detach().numpy()
        # u = u.detach().numpy()
        #
        # # loop over all rollouts
        # next_state = np.zeros(x.shape)
        # for k in range(next_state.shape[0]):
        #     next_state[k,:] = np.array([x[k,0] + 0.05 *  u[k,0],
        #         x[k,1] + 0.05 *  u[k,1],
        #         x[k,2]]
        #         ).reshape((3))
        # # print(f"the next states {next_state}")
        #
        # next_state_tensor = torch.tensor(next_state)

        return x #next_state_tensor

    targetState = State(pos=np.array([3,2,0]), ang_p=np.array([0,0,0]))

    def running_cost(x, u):
        x = x.detach().numpy()

        # print(f' shape of x {x.shape}, and of u {u.shape}')

        # loop over all rollouts
        next_state = np.zeros(x.shape)
        cost = np.zeros(next_state.shape[0])
        for k in range(next_state.shape[0]):
            next_state[k,:] = np.array([x[k,0] + 0.05 *  u[k,0],
                x[k,1] + 0.05 *  u[k,1],
                x[k,2]]
                ).reshape((3))
             # linear cost toward the target
            x_cost = abs(targetState.pos[0] - next_state[k,0])
            y_cost = abs(targetState.pos[1] - next_state[k,1])
            t_cost = abs(targetState.ang_p[2] - next_state[k,2])

            cost[k] = x_cost + y_cost + t_cost
            
            # print(f"the next states {next_state}")

        cost = torch.tensor(cost)
        # print(f'retrunign cost {cost.shape}')

        return cost


    # mppi_gym = mppi.MPPI(dynamics, running_cost, nx, noise_sigma, num_samples=N_SAMPLES, horizon=TIMESTEPS,
    #                      lambda_=lambda_, device=d, u_min=torch.tensor(ACTION_LOW, dtype=torch.double, device=d),
    #                      u_max=torch.tensor(ACTION_HIGH, dtype=torch.double, device=d))
    # total_reward, data = mppi.run_mppi(mppi_gym, env, train)

    # create controller with chosen parameters
    ctrl = mppi.MPPI(dynamics=dynamics,
            running_cost=running_cost,
            nx=nx,
            noise_sigma=torch.tensor([[5,0],[0,5]], device=d, dtype=torch.double),
            num_samples=250, # number of rolouts
            horizon=15,
            lambda_=1e-2,
            # device=d, 
            u_min=torch.tensor([-2, -2], dtype=torch.double, device=d),
            u_max=torch.tensor([2, 2], dtype=torch.double, device=d)
            )

    # add sensors
    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)


    # assuming you have a gym-like env
    for _ in range(n_steps):
        
        state = np.array([ob["joint_state"]["position"]])
        action[0:2] = ctrl.command(state).cpu().numpy()

        print(f'action: {action}')

        ob, reward, done, _ = env.step(action)
        # print(f"Observation: {ob}")


if __name__ == "__main__":
    main()
