from multiprocessing import Process, Pipe
import numpy as np
import gym
import urdfenvs.boxer_robot
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.rbrain import RBrain
from robot_brain.state import State

from environments.benchmark.benchmark_obstacles.obstacles import swap
from robot_brain.global_variables import DT

target_pos = np.array([0, 0, 0])
target_ang_p = np.array([0, 0, 0])

user_input_mode = False

def main(conn=None):
    env = gym.make("pointRobot-vel-v7", dt=DT, render=True)

    ob = env.reset()
    action = np.zeros(env.n())
    
    # env.add_obstacle(swap["small_duck"])
    env.add_obstacle(swap["small_box"])
    env.add_obstacle(swap["small_cylinder"])




    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)
    ob, _, _, _ = env.step(np.zeros(env.n()))

    for _ in range(10):

        ob, _, _, _ = env.step(action)


    env.add_target_ghost(swap["small_cylinder"].name(),  [2, 1, 0])
    env.add_target_ghost(swap["small_box"].name(),  [2, -1, 0])

    brain = None
    if not user_input_mode:
        brain = RBrain()
        brain.setup({
            "dt": DT,
            "robot_type": "boxer_robot",
            "target_state": State(),
            "objects_in_env": True,
            "objects": swap,
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
