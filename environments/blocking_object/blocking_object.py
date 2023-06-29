import numpy as np
import gym
from multiprocessing import Process, Pipe
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT

from environments.blocking_object.objects import wall1, wall2, wall3, blocking_object

USER_INPUT_MODE = True

def main(conn=None):
    """
    Point robot and objects which can interact with each other in the environment.

    Semantic brain goal: find out how interachtin with the objects goes

    """
    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())

    pos0 = np.array([1.0, 0.1])
    vel0 = np.array([0.0, 0.0])
    env.reset(pos=pos0, vel=vel0)

    n_steps = 10000

    # add objects
    objects = {blocking_object.name(): blocking_object, wall1.name(): wall1, wall2.name(): wall2, wall3.name(): wall3}
    env.add_obstacle(blocking_object)
    env.add_obstacle(wall1)
    env.add_obstacle(wall2)
    env.add_obstacle(wall3)

    # add sensors
    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)

    ob, reward, done, info = env.step(action)

    brain = None
    if not USER_INPUT_MODE:
        brain = RBrain()
        brain.setup({
            "dt": DT,
            "robot_type": robot_type,
            "objects_in_env": True,
            "default_action": np.array(np.zeros(2)),
            "task": [("robot", State(pos=np.array([-4.3212, -4, 0])))],
            "objects": objects,
            "env": env
        }, ob)

        brain.update(ob)

    for _ in range(n_steps):

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
        custom_on_press = {Key.down: np.array([1.0, 0.0]),
                           Key.up: np.array([-1.0, 0.0]),
                           Key.left: np.array([0, -1.0]),
                           Key.right: np.array([0, 1.0])}

        # responder.setup(defaultAction=np.array([0.0, 0.0]))
        responder.setup(custom_on_press=custom_on_press)

        # start child process which keeps responding/looping
        responder.start(p)

        # kill parent process
        p.kill()
