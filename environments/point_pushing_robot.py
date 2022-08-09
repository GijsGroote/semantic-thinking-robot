import numpy as np
import gym
import urdfenvs.point_robot_urdf
import urdfenvs.boxer_robot
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from multiprocessing import Process, Pipe
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.RBrain import RBrain
from robot_brain.RBrain import State
from environments.objects.spheres import sphere, sphere_small

user_input_mode = False

def main(conn=None):
    """
    Point robot and obstacles which can interact with each other in the environment.

    Semantic brain goal: find out how interachtin with the objects goes

    """
    dt = 0.05
    # env = gym.make('point-robot-urdf-vel-v0', dt=dt, render=True)
    env = gym.make('boxer-robot-vel-v0', dt=dt, render=True)

    pos0 = np.array([1.0, 0.1])
    vel0 = np.array([0.0, 0.0])
    env.reset(pos=pos0, vel=vel0)
     
    sensor = ObstacleSensor()
    env.add_sensor(sensor)
    defaultAction = np.array([0.0, 0.0])
    n_steps = 10000
    # env.add_walls()
    # env.add_shapes("GEOM_BOX", dim=[1,1,1], mass=15, poses_2d=[[-1,1,2]])
    
    # add obstacles
    env.add_obstacle(sphere)
    env.add_obstacle(sphere_small)

    ob, reward, done, info = env.step(defaultAction)

    print(ob)

    brain = RBrain()
    if not user_input_mode:
        # do the regular stuff, like begin the simulation, something like that
        brain.setup({
            "dt": dt,
            "defaultAction": defaultAction,
            "targetState": State(),
        }, ob)

    action = defaultAction
    for i in range(n_steps):

        if user_input_mode:
            conn.send({"request_action": True, "kill_child": False, "ob": ob})
            keyboard_data = conn.recv()
            action[0:2] = keyboard_data["action"]
        else:
            action = brain.respond()


        ob, reward, done, info = env.step(action)

        brain.update(ob)
        # print(ob)


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
