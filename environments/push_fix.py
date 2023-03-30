from multiprocessing import Process, Pipe
import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
import urdfenvs.boxer_robot
from pynput.keyboard import Key
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT

import pybullet as p
from motion_planning_env.box_obstacle import BoxObstacle

import math
from motion_planning_env.box_obstacle import BoxObstacle

# surrounded
surrounded = {
        "simpleBox1": BoxObstacle(name="simpleBox1", content_dict={
            "movable": True,
            "mass": 3,
            "type": "box",
            "orientation": [0, 0, 0],
            "color": [253, 1, 0],
            "position": [-2, 0, 0.6],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1.0},
            }),
        "simpleBox2": BoxObstacle(name="simpleBox2", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [160, 214, 54],
            "position": [2, 0, 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1},
            }),
        "simpleBox3": BoxObstacle(name="simpleBox3", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [238, 222, 4],
            "position": [0.9, math.sqrt(3), 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1},
            }),
        "simpleBox4": BoxObstacle(name="simpleBox4", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [255, 165, 0],
            "position": [-0.9, math.sqrt(3), 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1},
            }),
        "simpleBox5": BoxObstacle(name="simpleBox5", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [47, 1.5, 54],
            "position": [0.8, -math.sqrt(3), 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1},
            }),
        "simpleBox6": BoxObstacle(name="simpleBox6", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [51, 62, 212],
            "position": [-0.9, -math.sqrt(3), 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1},
            }),
}


def main():

    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())

    ob = env.reset()

    env.add_obstacle(surrounded["simpleBox1"])
    env.add_obstacle(surrounded["simpleBox2"])
    env.add_obstacle(surrounded["simpleBox3"])
    env.add_obstacle(surrounded["simpleBox4"])
    env.add_obstacle(surrounded["simpleBox5"])
    env.add_obstacle(surrounded["simpleBox6"])




    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)

    ob, reward, done, info = env.step(np.zeros(env.n()))

    brain = RBrain()

    brain.setup({
        "dt": DT,
        "robot_type": robot_type,
        "obstacles_in_env": True,
        "default_action": np.zeros(2),
        "task": [
            # ("robot", State(pos=np.array([4.12, 1.9, 0]))),
            ("simpleBox3", State(pos=np.array([0.92, 3.2, 0])))],
        "obstacles": surrounded,
        "env": env
        }, ob)

    for _ in range(1500):

        action[0:2] = brain.respond()
        ob, _, _, _ = env.step(action)
        brain.update(ob)


if __name__ == "__main__":
    main()
