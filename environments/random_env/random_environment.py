import math
import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT
from environments.random_env.random_objects import RandomObject

def main():
    """
    Randomly generated environment with a randomly assigned task
    for the pointrobot to solve.
    """

    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())

    env.reset()


    rand_obj_generator = RandomObject(
            grid_x_length=25,
            grid_y_length=15,
            min_dimension=0.2,
            max_dimension=2.0,
            max_weight=100,
            )

    obstacles, task = rand_obj_generator.create_random_objects_and_task(
            n_unmovable_obstacles= 4,
            n_movable_obstacles = 3,
            n_subtasks = 2)

    # add obstacles to environment
    for obstacle in obstacles.values():
        env.add_obstacle(obstacle)

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
        "task": task,
        "obstacles": obstacles,
        "env": env
    }, ob)

    brain.update(ob)

    for _ in range(10000):

        action[0:2] = brain.respond()
        ob, reward, done, info = env.step(action)
        brain.update(ob)

if __name__ == "__main__":
    main()
