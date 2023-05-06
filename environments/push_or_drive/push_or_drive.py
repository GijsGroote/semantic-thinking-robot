import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT

from environments.push_or_drive.objects import (
        blocking_object,
        wall1, wall2)

def main():

    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())
    env.reset()

    n_steps = 10000

    # add objects
    objects = {blocking_object.name(): blocking_object,
            wall1.name(): wall1,
            wall2.name(): wall2}
    # add walls
    env.add_obstacle(wall1)
    env.add_obstacle(wall2)
    # add objects
    env.add_obstacle(blocking_object)

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
        "task": [("robot", State(pos=np.array([0, 3, 0])))],
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
