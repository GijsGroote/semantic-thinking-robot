from multiprocessing import Process, Pipe
import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
import urdfenvs.boxer_robot # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT

from environments.objects.boxes import box, box2
from environments.objects.spheres import sphere
from environments.objects.cylinders import cylinder


user_input_mode = False

def main(conn=None):
    """
    Point robot and obstacles which can interact with each other in the environment.

    Semantic brain goal: find out how interachtin with the objects goes

    """
    # robot_type = "pointRobot-vel-v7"
    # robot_type = "pointRobot-acc-v7"
    robot_type = "boxerRobot-vel-v7"
    # robot_type = "boxerRobot-acc-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())

    pos0 = np.array([1.0, 0.1])
    vel0 = np.array([0.0, 0.0])
    env.reset(pos=pos0, vel=vel0)

    n_steps = 10000

    obstacles = {box.name(): box,
            box2.name(): box2,
            # sphere.name(): sphere
            cylinder.name(): cylinder}

    # add obstacles
    env.add_obstacle(box)
    env.add_obstacle(box2)
    # env.add_obstacle(sphere)
    env.add_obstacle(cylinder)

    ob, reward, done, info = env.step(action)

    for _ in range(n_steps):

        if user_input_mode:
            conn.send({"request_action": True, "kill_child": False, "ob": ob})
            keyboard_data = conn.recv()
            action[0:2] = keyboard_data["action"]

        ob, reward, done, info = env.step(action)

    if user_input_mode:
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
