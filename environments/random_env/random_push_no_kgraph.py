import math
import time
import numpy as np
import random
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT, GRID_X_SIZE, GRID_Y_SIZE
from environments.random_env.rand_objects import RandomObject
from helper_functions.figures import create_new_directory

from helper_functions.figures import (
        create_new_directory,
        create_time_plot)

def main():
    """
    Randomly generated environment with a randomly assigned task
    for the pointrobot to solve.
    """

    robot_type = "pointRobot-vel-v7"

    rand_obj_generator = RandomObject(
            grid_x_length=GRID_X_SIZE,
            grid_y_length=GRID_Y_SIZE,
            min_dimension=0.2,
            max_dimension=2.0,
            max_weight=1000)

    save_path = create_new_directory(dir_path="environments/random_env/data/push_data_no_kgraph")

    # with 23 turn blacklist off
    # seed 14, 20, 23, 24, 25, 26, 27, 29, 32 (exluding 15, 16, 17, 18, 19, 21, 22,28)
    random.seed(33)

    env = gym.make(robot_type, dt=DT, render=True)
    action = np.zeros(env.n())

    # TODO: Do not yet give the objects a locaaation
    rand_obj_generator.create_random_objects(
            n_unmovable_objects = 3,
            n_movable_objects = 5)

    for n_env in range(6):
        print(f"create environment number: {n_env}")

        objects = rand_obj_generator.reshuffle_env()
        task = rand_obj_generator.create_push_task(n_subtasks=1)
        env.reset()

        # add objects to environment
        for obj in objects.values():
            env.add_obstacle(obj)

        time.sleep(100000)
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
            "task": task,
            "kgraph": None,
            "objects": objects,
            "env": env,
            "n_env": n_env,
            "save_path": save_path,
        }, ob)

        brain.update(ob)

        try:
            for _ in range(10000):

                action[0:2] = brain.respond()
                ob, _, _, _ = env.step(action)
                brain.update(ob)

        except StopIteration as exc:

            print(f"Tear down this environment, we're done here because {exc}")
            continue

        print('times is up, try again')



if __name__ == "__main__":
    main()
