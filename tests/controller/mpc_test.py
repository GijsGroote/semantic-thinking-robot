import numpy as np
from casadi import vertcat

import urdfenvs.boxer_robot
import urdfenvs.point_robot_urdf

import gym
from robot_brain.controller.mpc.mpc import Mpc
from robot_brain.rbrain import State
from robot_brain.global_variables import DT

def main():
    env = gym.make('boxer-robot-vel-v0', dt=DT, render=True)
    # env = gym.make('pointRobotUrdf-vel-v0', dt=DT, render=True)

    default_action = np.array([0.0, 0.0, 0.0])
    n_steps = 1000
    ob = env.reset()

    pos = ob["joint_state"]["position"][0:2]
    vel = ob["joint_state"]["velocity"][0:2]
    current_state = State(pos=np.array([pos[0], pos[1], 0]),  vel=np.array([vel[0], vel[1], 0]))
    target_state = State(pos=np.array([1, 0, 0]), ang_p=np.array([0, 0, 0]))


    def dyn_model(x, u):
        dx_next = vertcat(
                x[0] + 0.05*np.cos(x[2]) * u[0],
                x[1] + 0.05*np.sin(x[2]) * u[0],
                x[2] + 0.05 * u[1])
        return dx_next


    mpc_controller = Mpc()
    mpc_controller.setup(dyn_model, current_state, target_state)


    ob, reward, done, info = env.step(default_action)


    for i in range(n_steps):

        pos = ob["joint_state"]["position"]
        vel = ob["joint_state"]["velocity"]
        current_state = State(pos=np.array([pos[0], pos[1], 0]),
                vel=np.array([vel[0], vel[1], 0]),
                ang_p=np.array([0, 0, pos[2]]),
                ang_v=np.array([0, 0, vel[2]]))

        action = mpc_controller.respond(current_state)

        ob, _, _, _ = env.step(action)

        if i==200:
            mpc_controller.visualise()

        if i == 50:
            mpc_controller.set_target_state(
                    State(pos=np.array([1, 0, 0]), ang_p=np.array([0, 0, 20])))

        if i == 100:
            mpc_controller.set_target_state(
                    State(pos=np.array([0, 0, 0]), ang_p=np.array([0, 0, 0])))

if __name__ == "__main__":
    main()





