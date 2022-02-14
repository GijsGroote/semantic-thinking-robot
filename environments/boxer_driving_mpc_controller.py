import numpy as np
import gym
import urdfenvs.boxerRobot
from robot_brain.RBrain import RBrain
from robot_brain.RBrain import State

pos = np.array([1.0, -2.0])
vel = np.array([0.0, 0.0])

target_pos = np.array([4.0, -2.0])
target_vel = np.array([0.0, 0.0])


def main():
    """
    Point robot which can drive around in its environment using an mpc controller.
    """
    dt = 0.25
    env = gym.make('boxer-robot-vel-v0', dt=dt, render=True)

    defaultAction = np.array([0.0, 0.0])
    n_steps = 1000

    ob = env.reset(pos=pos, vel=vel)
    t = 0

    # setup semantic brain
    brain = RBrain()


    ob, reward, done, info = env.step(defaultAction)

    # do the regular stuff, like begin the simulation, something like that
    brain.setup({"robot":
        {
            "pos": ob['x'],
            "vel": ob['xdot'],
        },
        "dt": dt
    })


    targetState = State(pos_x=target_pos[0], pos_y=target_pos[1], vel_x=0)  # drive to (0,0,0,0,0)
    print(targetState.toString())



    action = defaultAction

    for i in range(n_steps):
        t += env.dt()

        brain.update(ob)
        action = brain.respond()

        ob, reward, done, info = env.step(action)




if __name__ == '__main__':
    main()
