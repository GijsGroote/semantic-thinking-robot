import numpy as np
import gym
import urdfenvs.boxer_robot
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.RBrain import RBrain
from robot_brain.RBrain import State
from environments.objects.walls import surrounded 

target_pos = np.array([0, 0, 0])
target_ang_p = np.array([0, 0, 0])


def main():
    dt = 0.05
    env = gym.make('boxer-robot-vel-v0', dt=dt, render=True)

    n_steps = 1000
    ob = env.reset()
    
    # this should be done much easiers
    env.add_walls(dim=surrounded["wall1"]["dim"], poses_2d=surrounded["wall1"]["poses_2d"])
    env.add_walls(dim=surrounded["wall2"]["dim"], poses_2d=surrounded["wall2"]["poses_2d"])
    env.add_walls(dim=surrounded["wall3"]["dim"], poses_2d=surrounded["wall3"]["poses_2d"])
    env.add_walls(dim=surrounded["wall4"]["dim"], poses_2d=surrounded["wall4"]["poses_2d"])
    env.add_walls(dim=surrounded["wall5"]["dim"], poses_2d=surrounded["wall5"]["poses_2d"])
    env.add_walls(dim=surrounded["wall6"]["dim"], poses_2d=surrounded["wall6"]["poses_2d"])


    sensor = ObstacleSensor()
    env.add_sensor(sensor)
    ob, reward, done, _ = env.step(np.array([0.0, 0.0]))

    targetState = State(pos=target_pos, ang_p=target_ang_p)
    brain = RBrain()
    # do the regular stuff, like begin the simulation, something like that
    brain.setup({
        "dt": dt,
        "targetState": targetState,
        "obstacles": [surrounded]
    }, ob)


    for i in range(n_steps):

        action = brain.respond()

        ob, _, _, _ = env.step(action)
         
        # print(ob["obstacleSensor"])
        brain.update(ob)


if __name__ == '__main__':
    main()
