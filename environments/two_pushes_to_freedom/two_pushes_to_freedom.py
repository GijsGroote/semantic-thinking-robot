import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT

from environments.two_pushes_to_freedom.objects import (
        blocking_object1, blocking_object2, center_wall,
        wall1, wall2, wall3, wall4, wall5, wall6,
        wall7, wall8, wall9, wall10, wall11)

def main():

    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())
    env.reset()

    n_steps = 10000

    # add objects
    objects = {blocking_object1.name(): blocking_object1,
            blocking_object2.name(): blocking_object2,
            center_wall.name(): center_wall,
            wall1.name(): wall1,
            wall2.name(): wall2,
            wall3.name(): wall3,
            wall4.name(): wall4,
            wall5.name(): wall5,
            wall6.name(): wall6,
            wall7.name(): wall7,
            wall8.name(): wall8,
            wall9.name(): wall9,
            wall10.name(): wall10,
            wall11.name(): wall11}
    # add walls
    env.add_obstacle(wall1)
    env.add_obstacle(wall2)
    env.add_obstacle(wall3)
    env.add_obstacle(wall4)
    env.add_obstacle(wall5)
    env.add_obstacle(wall6)
    env.add_obstacle(wall7)
    env.add_obstacle(wall8)
    env.add_obstacle(wall9)
    env.add_obstacle(wall10)
    env.add_obstacle(wall11)
    # add objects
    env.add_obstacle(blocking_object1)
    env.add_obstacle(blocking_object2)
    env.add_obstacle(center_wall)

    # add sensors
    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)

    ob, reward, done, info = env.step(action)

    brain = RBrain()
    brain.setup({
        "dt": DT,
        "robot_type": robot_type,
        "objects_in_env": True,
        "default_action": np.array(np.zeros(2)),
        "task": [("robot", State(pos=np.array([-5, 3.5, 0])))],
        "objects": objects,
        "env": env
    }, ob)

    brain.update(ob)

    for _ in range(n_steps):

        action[0:2] = brain.respond()
        ob, reward, done, info = env.step(action)
        brain.update(ob)

if __name__ == "__main__":
    main()
