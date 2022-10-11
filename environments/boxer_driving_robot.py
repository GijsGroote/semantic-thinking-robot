import numpy as np
import gym
import urdfenvs.boxer_robot
from multiprocessing import Process, Pipe
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.rbrain import RBrain
from robot_brain.rbrain import State
# from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
# make obstacles in seperate file
# from MotionPlanningEnv.sphereObstacle import SphereObstacle
# from MotionPlanningEnv.boxObstacle import BoxObstacle

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

    default_action = np.array([0.0, 0.0])

    # trans_sphere_dict = {
    #     "movable": True,
    #     "mass": 1,
    #     "type": "sphere",
    #     "color": [1, 0, 0, 0.4],
    #     "position": [-1.0, 2.0, 1.8],
    #     "geometry": {"radius": 0.9},
    # }
    # sphere_dict = {
    #     "movable": True,
    #     "mass": 1,
    #     "type": "sphere",
    #     "color": [1, 0, 0, 1.0],
    #     "position": [2.0, 1.0, 1.8],
    #     "geometry": {"radius": 0.9},
    # }
    # trans_sphere= SphereObstacle(name="simpeSphere", content_dict=trans_sphere_dict)
    # sphere = SphereObstacle(name="simpeSphere", content_dict=sphere_dict)
    #
    # trans_box_dict = {
    #     "movable": True,
    #     "mass": 1,
    #     "type": "box",
    #     "color": [0, 1, 0, 0.4],
    #     "position": [2.0, -1.0, 1.8],
    #     "geometry": {"length": 0.3, "width": 0.4, "height": 0.5},
    # }
    # box_dict = {
    #     "movable": True,
    #     "mass": 1,
    #     "type": "box",
    #     "color": [0, 1, 0, 1.0],
    #     "position": [-3.0, 0, 1.8],
    #     "geometry": {"length": 0.3, "width": 0.4, "height": 0.5},
    # }
    #
    # trans_box= BoxObstacle(name="simpeBox", content_dict=trans_box_dict)
    # box = BoxObstacle(name="simpleBox", content_dict=box_dict)
    #
    # # env.add_obstacle(trans_sphere)
    # # env.add_obstacle(trans_box)
    # env.add_obstacle(sphere)
    # env.add_obstacle(box)

    # sensor = ObstacleSensor()
    # env.add_sensor(sensor)

    ob, reward, done, info = env.step(default_action)

    default_action = np.array([0.0, 0.0])
    n_steps = 100000

    # setup semantic brain
    brain = None
    if not user_input_mode:
        brain = RBrain()
        # do the regular stuff, like begin the simulation, something like that
        brain.setup({"dt": dt,
            "target_state": State(pos=np.array([1.9, 2.0, 4.0])),
        }, ob)


    # ob, reward, done, info = env.step(defaultAction)

    action = default_action

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
