import numpy as np
import gym
from multiprocessing import Process, Pipe
from urdfenvs.keyboardInput.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.RBrain import RBrain
from obstacles import sphereObst1, sphereObst2, urdfObst1, dynamicSphereObst1

user_input_mode = True


def main(conn):
    """
    Point robot and obstacles which can interact with each other in the environment.

    Semantic brain goal: find out how interachtin with the objects goes

    """
    env = gym.make('pointRobotUrdf-acc-v0', dt=0.01, render=True)
    defaultAction = np.array([0.0, 0.0])
    n_steps = 10000
    pos0 = np.array([1.0, 0.1])
    vel0 = np.array([0.0, 0.0])
    ob = env.reset(pos=pos0, vel=vel0)
    env.setWalls(limits=[[-5, -5], [3, 2]])
    t = 0


    # add obstacles
    env.addObstacle(sphereObst2)
    env.addObstacle(sphereObst2)

    # env.addObstacle(urdfObst1)

    action = defaultAction
    for i in range(n_steps):
        t += env.dt()

        conn.send({"request_action": True, "kill_child": False, "ob": ob})
        keyboard_data = conn.recv()
        action[0:2] = keyboard_data["action"]


        ob, reward, done, info = env.step(action)


    conn.send({"request_action": False, "kill_child": True})

if __name__ == '__main__':
    # setup multi threading with a pipe connection
    parent_conn, child_conn = Pipe()

    # create parent process
    p = Process(target=main, args=(parent_conn,))
    # start parent process
    p.start()

    if user_input_mode:

        # create Responder object
        responder = Responder(child_conn)

        # unlogical key bindings
        custom_on_press = {Key.left: np.array([-1.0, 0.0]),
                           Key.space: np.array([1.0, 0.0]),
                           Key.page_down: np.array([1.0, 1.0]),
                           Key.page_up: np.array([-1.0, -1.0])}

        responder.setup(defaultAction=np.array([0.0, 0.0]))
        # responder.setup(custom_on_press=custom_on_press)

        # start child process which keeps responding/looping
        responder.start(p)

    else:
        brain = RBrain(child_conn)

        brain.setup("we know not that much about the world")
        brain.start(p)

    # kill parent process
    p.kill()
