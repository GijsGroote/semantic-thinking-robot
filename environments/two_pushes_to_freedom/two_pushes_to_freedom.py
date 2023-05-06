import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT
from robot_brain.global_planning.kgraph.kgraph import KGraph
import math

from environments.two_pushes_to_freedom.objects import create_two_pushes_to_freedom_objects


def main():

    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())
    env.reset()


    kgraph = KGraph()

    # add obstacles
    obstacles = create_two_pushes_to_freedom_objects(rotate_by_theta=-0.3+math.pi/4)

    # add objects to environment
    for obst in obstacles.values():
        env.add_obstacle(obst)

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
        "task": [("robot", State(pos=np.array([-5, 3.5, 0])))],
        "obstacles": obstacles,
        "env": env,
        "n_env": 1,
        "kgraph": kgraph,
        "env": env
    }, ob)

    brain.update(ob)

    for _ in range(10000):

        action[0:2] = brain.respond()
        ob, reward, done, info = env.step(action)
        brain.update(ob)

if __name__ == "__main__":
    main()
