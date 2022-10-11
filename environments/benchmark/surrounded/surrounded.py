from multiprocessing import Process, Pipe
import numpy as np
import gym
import urdfenvs.boxer_robot
from pynput.keyboard import Key
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.global_planning.state import State
from robot_brain.global_variables import DT

from environments.benchmark.benchmark_obstacles.obstacles import surrounded

USER_INPUT_MODE = False

def main(conn=None):
    env = gym.make("boxer-robot-vel-v0", dt=DT, render=True)

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
            "robot_type": "boxer_robot",
            "target_state": State(),
            "obstacles_in_env": True,
            "obstacles": surrounded,
        }, ob)

    for i in range(1000):

        if i % 50 == 0: 
            brain.plot_occupancy_graph()
        if i == 300:
            brain.controller.set_target_state(State(pos=np.array([2,3,0])))
            brain.plot_occupancy_graph(save=True)
        if i == 500:
            brain.controller.set_target_state(State(pos=np.array([-8,1,0])))
            brain.plot_occupancy_graph()
        if i == 900:
            brain.plot_occupancy_graph(save=True)


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
