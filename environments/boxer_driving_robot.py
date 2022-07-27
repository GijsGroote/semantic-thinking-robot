import numpy as np
import gym
import urdfenvs.boxer_robot
from multiprocessing import Process, Pipe
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.RBrain import RBrain
from robot_brain.RBrain import State
# from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from environments.obstacles import sphereObst2

user_input_mode = True

def main(conn=None):
    """
    Point robot which can drive around in its environment.

    Semantic brain goal: find out that the robot can drive toward specified goals.

    """
    dt = 0.02
    # env = gym.make('pointRobotUrdf-acc-v0', dt=0.05, render=True)
    env = gym.make('boxer-robot-vel-v0', dt=dt, render=True)
    ob = env.reset()

    defaultAction = np.array([0.0, 0.0])

    # sensor = ObstacleSensor()
    # env.add_sensor(sensor)

    env.add_obstacle(sphereObst2)
    ob, reward, done, _ = env.step(defaultAction)


    defaultAction = np.array([0.0, 0.0])
    n_steps = 1000
    
    # setup semantic brain
    brain = None
    if not user_input_mode:
        brain = RBrain()
        # do the regular stuff, like begin the simulation, something like that
        brain.setup({"dt": dt,
            "targetState": State(pos=np.array([1.9, 2.0, 4.0])),
        }, ob)


    # ob, reward, done, info = env.step(defaultAction)

    action = defaultAction

    for _ in range(n_steps):

        if user_input_mode:
            conn.send({"request_action": True, "kill_child": False, "ob": ob})
            keyboard_data = conn.recv()
            action[0:2] = keyboard_data["action"]
        else:
            action = brain.respond()

        ob, reward, done, info = env.step(action)

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
        custom_on_press = {Key.down: np.array([-1.0, 0.0]),
                           Key.up: np.array([1.0, 0.0]),
                           Key.left: np.array([1.0, 1.0]),
                           Key.right: np.array([1.0, -1.0])}

        # responder.setup(defaultAction=np.array([0.0, 0.0]))
        responder.setup(custom_on_press=custom_on_press)

        # start child process which keeps responding/looping
        responder.start(p)

        # kill parent process
        p.kill()
