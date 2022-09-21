from multiprocessing import Process, Pipe
import numpy as np
import gym
import urdfenvs.boxer_robot
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.rbrain import RBrain
from robot_brain.rbrain import State

from environments.benchmark.benchmark_obstacles.obstacles import duck_small, box_small


target_pos = np.array([0, 0, 0])
target_ang_p = np.array([0, 0, 0])

user_input_mode = True

def main(conn=None):
    dt = 0.05
    env = gym.make("boxer-robot-vel-v0", dt=dt, render=True)

    ob = env.reset()
    
    env.add_obstacle(duck_small)
    env.add_obstacle(box_small)

    sensor = ObstacleSensor()
    env.add_sensor(sensor)
    ob, _, _, _ = env.step(np.array([0.0, 0.0]))

    brain = None
    if not user_input_mode:
        targetState = State(pos=target_pos, ang_p=target_ang_p)
        brain = RBrain()
        brain.setup({
            "dt": dt,
            "targetState": targetState,
            "obstacles": []
        }, ob)

    for _ in range(1000):

        if user_input_mode:
            conn.send({"request_action": True, "kill_child": False, "ob": ob})
            keyboard_data = conn.recv()
            action = keyboard_data["action"]
            ob, _, _, _ = env.step(action)
        else:
            action = brain.respond()
            ob, _, _, _ = env.step(action)
            brain.update(ob)


    if user_input_mode:
        conn.send({"request_action": False, "kill_child": True})



if __name__ == "__main__":

    if not user_input_mode:
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
                           Key.left: np.array([0.0, 1.0]),
                           Key.right: np.array([0.0, -1.0])}

        # responder.setup(defaultAction=np.array([0.0, 0.0]))
        responder.setup(custom_on_press=custom_on_press)

        # start child process which keeps responding/looping
        responder.start(p)

        # kill parent process
        p.kill()
