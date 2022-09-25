from multiprocessing import Process, Pipe
import numpy as np
import gym
import urdfenvs.boxer_robot
from pynput.keyboard import Key
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.rbrain import State
from robot_brain.global_variables import DT

from environments.benchmark.benchmark_obstacles.obstacles import surrounded

target_pos = np.array([0, 0, 0])
target_ang_p = np.array([0, 0, 0])

USER_INPUT_MODE = True

def main(conn=None):
    env = gym.make("boxer-robot-vel-v0", dt=DT, render=True)

    action = np.zeros(env.n())

    ob = env.reset()

    env.add_obstacle(surrounded["box1"])
    env.add_obstacle(surrounded["box2"])
    env.add_obstacle(surrounded["box3"])
    env.add_obstacle(surrounded["box4"])
    env.add_obstacle(surrounded["box5"])
    env.add_obstacle(surrounded["box6"])


    brain = None
    if not USER_INPUT_MODE:
        sensor = ObstacleSensor()
        env.add_sensor(sensor)
        target_state = State(pos=target_pos, ang_p=target_ang_p)
        brain = RBrain()
        brain.setup({
            "dt": DT,
            "target_state": target_state,
            "obstacles": []
        }, ob)

    for _ in range(1000):

        if USER_INPUT_MODE:
            conn.send({"request_action": True, "kill_child": False, "ob": ob})
            keyboard_data = conn.recv()
            action[0:2] = keyboard_data["action"]
            ob, _, _, _ = env.step(action)

        else:
            action = brain.respond()
            ob, _, _, _ = env.step(action)
            brain.update(ob)


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
