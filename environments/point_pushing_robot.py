import numpy as np
import gym
import urdfenvs.boxerRobot
from multiprocessing import Process, Pipe
from urdfenvs.keyboardInput.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.RBrain import RBrain
from obstacles import sphereObst1, sphereObst2, urdfObst1, dynamicSphereObst1

user_input_mode = True



def main(conn=None):
    """
    Point robot and obstacles which can interact with each other in the environment.

    Semantic brain goal: find out how interachtin with the objects goes

    """
    # env = gym.make('pointRobotUrdf-acc-v0', dt=0.05, render=True)
    env = gym.make('boxer-robot-vel-v0', dt=0.05, render=True)
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

    brain = RBrain()
    if not user_input_mode:
        # do the regular stuff, like begin the simulation, something like that
        brain.setup({"robot":
            {
                "pos": pos0,
                "vel": vel0,
            }
        })



    action = defaultAction
    for i in range(n_steps):
        t += env.dt()

        if user_input_mode:
            conn.send({"request_action": True, "kill_child": False, "ob": ob})
            keyboard_data = conn.recv()
            action[0:2] = keyboard_data["action"]
        else:
            action = brain.respond()

        ob, reward, done, info = env.step(action)


    conn.send({"request_action": False, "kill_child": True})

if __name__ == '__main__':

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
        custom_on_press = {Key.left: np.array([-1.0, 0.0]),
                           Key.space: np.array([1.0, 0.0]),
                           Key.page_down: np.array([1.0, 1.0]),
                           Key.page_up: np.array([-1.0, -1.0])}

        responder.setup(defaultAction=np.array([0.0, 0.0]))
        # responder.setup(custom_on_press=custom_on_press)

        # start child process which keeps responding/looping
        responder.start(p)

        # kill parent process
        p.kill()
