import os
from multiprocessing import Process, Pipe
import numpy as np
import gym
from dashboard.app import stop_dash_server
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
import urdfenvs.boxer_robot
from pynput.keyboard import Key
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.state import State
from robot_brain.global_variables import (
        CREATE_SERVER_DASHBOARD,
        DT,
        POINT_ROBOT_RADIUS,
        LOG_METRICS,
        SAVE_LOG_METRICS,
        PROJECT_PATH)

from robot_brain.object import Object, FREE, MOVABLE, UNKNOWN, UNMOVABLE

from helper_functions.figures import create_time_plot, create_prediction_error_plot, create_new_directory
from environments.benchmark.surrounded.objects import surrounded

USER_INPUT_MODE = False

def main(conn=None):

    robot_type = "pointRobot-vel-v7"

    env = gym.make(robot_type, dt=DT, render=True)

    kgraph = KGraph()


    # create new directory for data
    save_path = create_new_directory(dir_path= "environments/benchmark/surrounded/data/")


        # assert POINT_ROBOT_RADIUS < 0.31,\
    #     f"POINT_ROBOT_RADIUS must be smaller than TODO, otherwise it will end up in obstacle space"

    # try the same task multiple times
    for i in range(3):
        print(f'starting environment: {i}')

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
                "objects_in_env": True,
                "default_action": np.zeros(2),
                "task": [
                    (surrounded["simpleBox3"].name(), State(pos=np.array([-1, 3.5, 0]))),
                    # ("robot", State(pos=np.array([0, 4, 0])))
                    ],
                "objects": surrounded,
                "env": env,
                "n_env": i,
                "kgraph": kgraph,
                "save_path": save_path,
                }, ob)

        try:
            for _ in range(10000):

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

        except StopIteration as exc:

            print(f"Tear down this environment, we're done here because {exc}")
            continue

        print('times is up, try again')

        if CREATE_SERVER_DASHBOARD:
            stop_dash_server(brain.dash_app)

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
