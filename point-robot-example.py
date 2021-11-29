import gym
import gijsRobot
from gym_envs_urdf.sensors.lidar import Lidar
import numpy as np


def main():
    # initializing gijsRobot environment
    env = gym.make('gijsRobot-acc-v0', dt=0.05, render=True)
    # # adding a lidar sensor
    # lidar = Lidar(4, nbRays=4)
    # env.addSensor(lidar)


    defaultAction = np.array([0.0, 0.0])
    n_episodes = 1
    n_steps = 100000
    cumReward = 0.0

    # initial position robot
    pos0 = np.array([1.0, 0.1])
    vel0 = np.array([1.0, 0.0])

    # reset loop
    for e in range(n_episodes):
        ob = env.reset(pos=pos0, vel=vel0)
        env.setWalls(limits=[[-3, -3], [3, 3]])
        print("Starting episode")
        t = 0
        # main loop
        for i in range(n_steps):
            t += env.dt()
            action = defaultAction
            ob, reward, done, info = env.step(action)
            # print(done)
            cumReward += reward


if __name__ == '__main__':
    main()
