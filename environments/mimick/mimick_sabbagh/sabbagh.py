from multiprocessing import Process, Pipe
import numpy as np
import gym

import os
import math
from motion_planning_env.urdf_obstacle import UrdfObstacle
from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

import urdfenvs.boxer_robot
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.rbrain import RBrain
from robot_brain.state import State


from dashboard.app import stop_dash_server
from robot_brain.global_variables import CREATE_SERVER_DASHBOARD, DT
from helper_functions.figures import create_time_plot, create_prediction_error_plot, create_new_directory

from robot_brain.global_planning.kgraph.kgraph import KGraph


from helper_functions.figures import (
        create_new_directory,
        create_time_plot,
        create_prediction_error_plot,
        create_full_prediction_error_plot)


box_dict = {
    "color": [44, 95, 45],
    "position": [2, 1, 0.3],
    "type": "box",
    "geometry": {"length": 0.9, "width": 0.9, "height": 0.6},
}


box = BoxObstacle(name="box", content_dict=box_dict)


def main():
    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    kgraph = KGraph()

    # create new directory for data
    save_path = create_new_directory(dir_path="logger/logs")

    # try the same task multiple times
    for i in range(1):
        print(f'starting environment: {i}')

        action = np.zeros(env.n())
        ob = env.reset()

        env.add_obstacle(box)

        sensor = ObstacleSensor()
        sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
        env.add_sensor(sensor)
        ob, _, _, _ = env.step(np.zeros(env.n()))


        brain = RBrain()
        brain.setup({
            "dt": DT,
            "robot_type": robot_type,
            "objects_in_env": True,
            "default_action": np.zeros(2),
            "objects": {box.name(): box},
            "task": [
                    (box.name(), State(pos=np.array([2, 4, 0])))],
            "env": env,
            "n_env": i,
            "kgraph": kgraph,
            "save_path": save_path,
        }, ob)

        try:
            for _ in range(10000):
                action[0:2] = brain.respond()
                ob, _, _, _ = env.step(action)
                brain.update(ob)

        except StopIteration as exc:

            print(f"Tear down this environment, we're done here because {exc}")
            continue

        print('times is up, try again')

        if CREATE_SERVER_DASHBOARD:
            stop_dash_server(brain.dash_app)

    create_time_plot(data_path=save_path)
    create_prediction_error_plot(data_path=save_path)
    create_full_prediction_error_plot(data_path=save_path)

if __name__ == "__main__":
    main()
