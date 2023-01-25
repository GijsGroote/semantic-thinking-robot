import math
import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT

from environments.blocking_object.objects import wall1, wall2, wall3, blocking_object

def main():
    """
    Point robot and obstacles which can interact with each other in the environment.

    Semantic brain goal: find out how interachtin with the objects goes

    """
    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())

    pos0 = np.array([1.0, 0.1])
    vel0 = np.array([0.0, 0.0])
    env.reset(pos=pos0, vel=vel0)

    n_steps = 10000

    # add obstacles
    obstacles = {blocking_object.name(): blocking_object, wall1.name(): wall1, wall2.name(): wall2, wall3.name(): wall3}
    env.add_obstacle(blocking_object)
    env.add_obstacle(wall1)
    env.add_obstacle(wall2)
    env.add_obstacle(wall3)

    # add sensors
    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)

    ob, reward, done, info = env.step(action)

    brain = RBrain()
    brain.setup({
        "dt": DT,
        "robot_type": robot_type,
        "obstacles_in_env": True,
        "default_action": np.array(np.zeros(2)),
        "task": [("robot", State(pos=np.array([-3.3212, -2, 0])))],
        "obstacles": obstacles,
        "env": env
    }, ob)

    brain.update(ob)

    for _ in range(n_steps):

        action[0:2] = brain.respond()
        ob, reward, done, info = env.step(action)
        brain.update(ob)

if __name__ == "__main__":
    main()
