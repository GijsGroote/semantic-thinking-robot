from multiprocessing import Process, Pipe
import math
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


from motion_planning_env.box_obstacle import BoxObstacle

from motion_planning_env.cylinder_obstacle import CylinderObstacle



user_input_mode = False

def main(conn=None):
    """
    Point robot and obstacles which can interact with each other in the environment.

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

    box_dict = {
            "movable": True,
            "orientation": [0, 0, 0],
            "mass": 3,
            "type": "box",
            "color": [0/255, 255/255, 0/255, 1],
            "position": [-1.3, 3.5, 0.5],
            "geometry": {"length": 2.9, "width": 1.6, "height": 0.3},
            }

    cylinder_dict = {
            "movable": True,
            "mass": 1,
            "type": "cylinder",
            "color": [0/255, 255/255, 0/255, 1],
            "position": [-2.0, 1.7, 0.5],
            "geometry": {"radius": 0.62, "height": 0.25},
            }

    cylinder = CylinderObstacle(name="simple_cilinder", content_dict=cylinder_dict)
    box = BoxObstacle(name="simple_box", content_dict=box_dict)

    obstacles = {box.name(): box,
            cylinder.name(): cylinder}

    # add obstacles
    env.add_obstacle(box)
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
        "obstacles_in_env": True,
        "default_action": np.array(np.zeros(2)),
        "task": [#(box.name(), State(pos=np.array([3.3212, 2, 0]))),
            # (box.name(), State(pos=np.array([-5, -1, 0]))),
            ("robot", State(pos=np.array([-3.3212, 3, 0]))),
            ("robot", State(pos=np.array([-3.4212, 3, 0]))),
            ("robot", State(pos=np.array([0.2, 0.1, -math.pi/2]))),
            ("robot", State(pos=np.array([3.3212, 2.80, 0]))),
            ("robot", State(pos=np.array([4,-4,0]))),
            # ("robot", State(pos=np.array([-4, -4, 0]))),
            ],
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
