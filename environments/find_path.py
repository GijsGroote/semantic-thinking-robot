from multiprocessing import Process, Pipe
import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
import urdfenvs.boxer_robot # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT

from environments.objects.boxes import box, box2, box3
from environments.objects.spheres import sphere
from environments.objects.cylinders import cylinder

def main():
    """
    Point robot and objects which can interact with each other in the environment.

    Semantic brain goal: find out how interachtin with the objects goes

    """
    robot_type = "pointRobot-vel-v7"
    # robot_type = "pointRobot-acc-v7"
    # robot_type = "boxerRobot-vel-v7"
    # robot_type = "boxerRobot-acc-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())

    pos0 = np.array([1.0, 0.1])
    vel0 = np.array([0.0, 0.0])
    env.reset(pos=pos0, vel=vel0)

    n_steps = 10000

    objects = {box.name(): box,
            box2.name(): box2,
            box3.name(): box3,
            sphere.name(): sphere,
            cylinder.name(): cylinder}

    # add objects
    env.add_obstacle(box)
    env.add_obstacle(box2)
    env.add_obstacle(box3)
    env.add_obstacle(sphere)
    env.add_obstacle(cylinder)

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
        # "task": [("robot", State(pos=np.array([-3.3212, 3.80, 0]))),# (box, State(pos=np.array([-2.3, -3.80, 0]))),
        #     ("robot", State(pos=np.array([3.3212, 2.80, 0]))),
        #     ("robot", State(pos=np.array([3,0,0]))),
        #     ("robot", State(pos=np.array([1, 2.2, 0]))),
        #     ],
        "task": [("robot", State(pos=np.array([-2.12, -4.5, 0]))),
            # ("robot", State(pos=np.array([-3.3212, 3, 0]))),
            # ("robot", State(pos=np.array([3.3212, 2, 0]))),
            # ("robot", State(pos=np.array([3.3212, -2, 0]))),
            # ("robot", State(pos=np.array([-2.3212, -1, 0]))),
            # ("robot", State(pos=np.array([3.3212, 2.80, 0]))),
            # ("robot", State(pos=np.array([3,0,0]))),
            # ("robot", State(pos=np.array([1, 2.2, 0]))),
            ],
        "objects": objects
    }, ob)

    brain.update(ob)

    for i in range(n_steps):

        action[0:2] = brain.respond()
        ob, reward, done, info = env.step(action)

        brain.update(ob)



if __name__ == "__main__":
        main()
