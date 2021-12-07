import gym
import gijsRobot
# from gym_envs_urdf.sensors.lidar import Lidar
from robot_brain import RBrain
from robot_brain.main_brain import main_b
from multiprocessing import Process, Pipe
import numpy as np


def main(child_conn):

    # initializing gijsRobot environment
    env = gym.make('gijsRobot-acc-v0', dt=0.01, render=True)
    # # adding a lidar sensor
    # lidar = Lidar(4, nbRays=4)
    # env.addSensor(lidar)


    defaultAction = np.array([0.0, 0.0])
    n_episodes = 1  # semantic brain should not need multiple episodes
    n_steps = 100000
    cumReward = 0.0 # semantic brain does not need a reward

    # initial position robot
    pos0 = np.array([1.0, 0.1])
    vel0 = np.array([0.0, 0.0])

    # reset loop
    for e in range(n_episodes):
        ob = env.reset(pos=pos0, vel=vel0)
        env.setWalls(limits=[[-3, -3], [3, 3]])
        print("Starting episode")
        t = 0
        # main loop
        for i in range(n_steps):
            t += env.dt()
            # receive data from parent connection
            data = child_conn.recv()

            action = np.array([data["x"], data["y"]])
            # action = defaultAction
            ob, reward, done, info = env.step(action)

            # send information to parent process
            child_conn.send({"robot_accepts_input": True, "ob": ob})
            # print(done)

            # cumReward += reward


if __name__ == '__main__':
    # setup multi threading with a pipe connection
    parent_conn, child_conn = Pipe()
    p = Process(target=main, args=(child_conn, ))
    # start child process
    p.start()

    # create robot brain
    main_b(p, parent_conn)



