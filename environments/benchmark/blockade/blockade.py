import numpy as np
import time
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from multiprocessing import Process, Pipe
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import CREATE_SERVER_DASHBOARD, DT
from dashboard.app import stop_dash_server
from robot_brain.global_planning.kgraph.kgraph import KGraph

from environments.benchmark.benchmark_obstacles.obstacles import blockade_obstacles
from helper_functions.figures import create_new_directory

USER_INPUT_MODE = True

def main(conn=None):
    """
    Point robot which can drive around in its environment using a mpc controller.
    """
    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)
    kgraph = KGraph()

    # create new directory for data
    # save_path = create_new_directory(dir_path="environments/benchmark/blockade/data/")

    action = np.zeros(env.n())
    ob = env.reset()
    env.add_obstacle(blockade_obstacles["simpleCylinder"])
    env.add_obstacle(blockade_obstacles["simpleBox"])
    env.add_obstacle(blockade_obstacles["wall1"])
    env.add_obstacle(blockade_obstacles["wall2"])
    env.add_obstacle(blockade_obstacles["wall3"])

    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)

    ob, reward, done, info = env.step(np.zeros(env.n()))


    brain = None
    if not USER_INPUT_MODE:
        brain = RBrain()

        brain.setup({
            "dt": DT,
            "robot_type": robot_type,
            "objects_in_env": True,
            "default_action": np.zeros(2),
            "task": [
                (blockade_obstacles["simpleBox"].name(), State(pos=np.array([3, 0, 0]))),
                ],
            "objects": blockade_obstacles,
            "env": env,
            "n_env": 0,
            "kgraph": kgraph,
            # "save_path": save_path,
            "save_path": None,
            }, ob)

        brain.update(ob)

    for _ in range(10000):

        if USER_INPUT_MODE:
            conn.send({"request_action": True, "kill_child": False, "ob": ob})
            keyboard_data = conn.recv()
            action[0:2] = keyboard_data["action"]
        else:
            brain.update(ob)
            action = brain.respond()

        ob, reward, done, info = env.step(action)

    if USER_INPUT_MODE:
        conn.send({"request_action": False, "kill_child": True})


if __name__ == '__main__':

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

        # unlogical key bindings
        custom_on_press = {Key.down: np.array([-1.0, 0.0]),
                           Key.up: np.array([1.0, 0.0]),
                           Key.left: np.array([0, 1.0]),
                           Key.right: np.array([0, -1.0])}

        # responder.setup(defaultAction=np.array([0.0, 0.0]))
        responder.setup(custom_on_press=custom_on_press)

        # start child process which keeps responding/looping
        responder.start(p)

        # kill parent process
        p.kill()
