import numpy as np
import gym
import urdfenvs.boxer_robot
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.RBrain import RBrain
from robot_brain.RBrain import State

target_pos = np.array([1, 1, 0])
target_ang_p = np.array([0, 0, 1])
controller = "mpc"


def main():
    """
    Point robot which can drive around in its environment using a mpc controller.
    """
    dt = 0.1
    env = gym.make('boxer-robot-vel-v0', dt=dt, render=True)

    defaultAction = np.array([0.0, 0.0])
    n_steps = 1000
    ob = env.reset()
    sensor = ObstacleSensor()
    env.add_sensor(sensor)



    ob, reward, done, info = env.step(defaultAction)

    targetState = State(pos=target_pos, ang_p=target_ang_p)
    brain = RBrain()
    # do the regular stuff, like begin the simulation, something like that
    brain.setup({
        "dt": dt,
        "defaultAction": defaultAction,
        "targetState": targetState,
        "controller": controller,
        "additionalParameters??": True
    }, ob)

    action = defaultAction

    for i in range(n_steps):

        action = brain.respond()

        ob, reward, done, info = env.step(action)

        brain.update(ob)


if __name__ == '__main__':
    main()
