from multiprocessing import Process, Pipe
import numpy as np
import gym
from pynput.keyboard import Key
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
import urdfenvs.boxer_robot
from robot_brain.rbrain import RBrain
from robot_brain.planning.state import State

from environments.objects.urdf_objects import urdf_duck
from environments.benchmark.benchmark_obstacles.obstacles import pushable_cube, dead_end
# from environments.objects.walls import dead_end, pushable_cube

target_pos = np.array([0, 0, 0])
target_ang_p = np.array([0, 0, 0])

USER_INPUT_MODE = False 


def main(conn=None):
    """
    Point robot which can drive around in its environment using a mpc controller.
    """
    dt = 0.05
    env = gym.make('boxer-robot-vel-v0', dt=dt, render=True)

    ob = env.reset()
    env.add_obstacle(urdf_duck)

    # this should be done much easiers
    env.add_obstacle(pushable_cube)
    # env.add_walls(dim=dead_end["wall1"]["dim"], poses_2d=dead_end["wall1"]["poses_2d"])
    # env.add_walls(dim=dead_end["wall2"]["dim"], poses_2d=dead_end["wall2"]["poses_2d"])
    # env.add_walls(dim=dead_end["wall3"]["dim"], poses_2d=dead_end["wall3"]["poses_2d"])

    # env.add_walls(dim=pushable_cube["dim"], poses_2d=pushable_cube["poses_2d"])

    brain = None
    if not USER_INPUT_MODE:
        sensor = ObstacleSensor()
        env.add_sensor(sensor)
        target_state = State(pos=target_pos, ang_p=target_ang_p)
        brain = RBrain()
        brain.setup({
            "dt": dt,
            "target_state": target_state,
            "obstacles": []
        }, ob)

    target_state = State(pos=target_pos, ang_p=target_ang_p)
    brain = RBrain()
    # do the regular stuff, like begin the simulation, something like that
    brain.setup({
        "dt": dt,
        "target_state": target_state,
    }, ob)

    for _ in range(1000):

        if USER_INPUT_MODE:
            conn.send({"request_action": True, "kill_child": False, "ob": ob})
            keyboard_data = conn.recv()
            action = keyboard_data["action"]
            ob, _, _, _ = env.step(action)
        else:
            action = brain.respond()
            ob, _, _, _ = env.step(action)
            brain.update(ob)


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
