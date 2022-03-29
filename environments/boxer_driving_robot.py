import numpy as np
import gym
import urdfenvs.boxer_robot
from multiprocessing import Process, Pipe
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.RBrain import RBrain
from robot_brain.RBrain import State
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor


user_input_mode = False

def main(conn=None):
    """
    Point robot which can drive around in its environment.

    Semantic brain goal: find out that the robot can drive toward specified goals.

    """
    dt = 0.05
    # env = gym.make('pointRobotUrdf-acc-v0', dt=0.05, render=True)
    env = gym.make('boxer-robot-vel-v0', dt=dt, render=True)

    ob = env.reset()
    env.set_walls(limits=[[-5, -5], [3, 2]])

    sensor = ObstacleSensor()
    env.add_sensor(sensor)

    defaultAction = np.array([0.0, 0.0])
    n_steps = 1000



    # setup semantic brain
    brain = None
    if not user_input_mode:
        brain = RBrain()
        # do the regular stuff, like begin the simulation, something like that
        brain.setup({"dt": dt,
                     "targetState": State(),
        }, ob)
        targetState = State()  # drive to (0,0,0,0,0)


    # ob, reward, done, info = env.step(defaultAction)

    action = defaultAction

    for i in range(n_steps):

        if user_input_mode:
            conn.send({"request_action": True, "kill_child": False, "ob": ob})
            keyboard_data = conn.recv()
            action[0:2] = keyboard_data["action"]
        else:
            action = brain.respond()

        ob, reward, done, info = env.step(action)
        print(ob['obstacleSensor']['2'])
        print(ob)

    if user_input_mode:
        conn.send({"request_action": False, "kill_child": True})

        brain.update(ob)
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
