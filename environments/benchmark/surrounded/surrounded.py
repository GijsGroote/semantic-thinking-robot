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
from robot_brain.global_planning.kgraph.kgraph import KGraph

from environments.benchmark.surrounded.objects import surrounded

USER_INPUT_MODE = False

def main(conn=None):

    robot_type = "pointRobot-vel-v7"


    kgraph = KGraph()


    # try the same task multiple times
    for i in range(4):
        print(f'starting environment: {i}')

        env = gym.make(robot_type, dt=DT, render=True)

        action = np.zeros(env.n())

        ob = env.reset()

        env.add_obstacle(surrounded["simpleBox1"])
        env.add_obstacle(surrounded["simpleBox2"])
        env.add_obstacle(surrounded["simpleBox3"])
        env.add_obstacle(surrounded["simpleBox4"])
        env.add_obstacle(surrounded["simpleBox5"])
        env.add_obstacle(surrounded["simpleBox6"])

        brain = None

        if not USER_INPUT_MODE:

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
                "task": [("robot", State(pos=np.array([-4.12, 0.9, 0])))],
                "obstacles": surrounded,
                "env": env,
                "kgraph": kgraph,
                }, ob)

        try:
            for _ in range(1500):

                ob, _, _, _ = env.step(action)

                if USER_INPUT_MODE:
                    conn.send({"request_action": True, "kill_child": False, "ob": ob})
                    keyboard_data = conn.recv()
                    action[0:2] = keyboard_data["action"]
                    ob, _, _, _ = env.step(action)

                else:
                    action[0:2] = brain.respond()
                    ob, _, _, _ = env.step(action)
                    brain.update(ob)

        except ValueError as exc:
            print(f'exception in the environment {exc} try again')
            continue

        print('enought try again')

    if USER_INPUT_MODE:
        conn.send({"request_action": False, "kill_child": True})

if __name__ == "__main__":

    if not USER_INPUT_MODE:
        main()

    else:
        # setup multi threading with a pipe connection
        parent_conn, child_conn = Pipe()

        # create parent process
        p = Process(target=main, args=(parent_conn,))
        # start parent process
        p.start()

        # create Responder object
        responder = Responder(child_conn)

        # logical key bindings
        custom_on_press = {Key.down: np.array([-1, 0]),
                           Key.up: np.array([1, 0]),
                           Key.left: np.array([0, 1]),
                           Key.right: np.array([0, -1])}

        # responder.setup(defaultAction=np.array([0, 0]))
        responder.setup(custom_on_press=custom_on_press)

        # start child process which keeps responding/looping
        responder.start(p)

        # kill parent process
        p.kill()
