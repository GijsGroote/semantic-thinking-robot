import numpy as np
import gym
import urdfenvs.boxerRobot
from robot_brain.RBrain import RBrain
from robot_brain.RBrain import State

pos = np.array([1.0, -2.0])
vel = np.array([0.0, 0.0])

target_pos = np.array([0.0, 0.0])
target_vel = np.array([0.0, 0.0])


def main():
    """
    Point robot which can drive around in its environment using an mpc controller.
    """
    dt = 0.1
    env = gym.make('boxer-robot-vel-v0', dt=dt, render=True)

    defaultAction = np.array([0.0, 0.0])
    n_steps = 1000
    ob = env.reset(pos=pos, vel=vel)

    # setup semantic brain
    brain = RBrain()

    ob, reward, done, info = env.step(defaultAction)

    targetState = State(pos=target_pos, vel=target_vel)  # drive to (0,0,0,0,0)
    # do the regular stuff, like begin the simulation, something like that
    brain.setup({
        "dt": dt,
        "defaultAction": defaultAction,
        "targetState": targetState,
        "additionalParameters??": True
    }, ob)




    action = defaultAction

    for i in range(n_steps):

        action = brain.respond()
        print("action; {}".format(action))

        ob, reward, done, info = env.step(action)

        brain.update(ob)


if __name__ == '__main__':
    main()
