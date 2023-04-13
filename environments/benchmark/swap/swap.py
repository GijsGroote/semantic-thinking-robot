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


from dashboard.app import stop_dash_server
from environments.benchmark.benchmark_obstacles.obstacles import swap
from robot_brain.global_variables import CREATE_SERVER_DASHBOARD, DT
from helper_functions.figures import create_time_plot, create_prediction_error_plot, create_new_directory

from robot_brain.global_planning.kgraph.kgraph import KGraph

user_input_mode = False

def main(conn=None):
    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    kgraph = KGraph()

    # create new directory for data
    save_path = create_new_directory(dir_path="environments/benchmark/swap/data/")

    # try the same task multiple times
    for i in range(3):
        print(f'starting environment: {i}')

        action = np.zeros(env.n())
        ob = env.reset()

        # env.add_obstacle(swap["small_duck"])
        env.add_obstacle(swap["small_box"])
        env.add_obstacle(swap["small_cylinder"])

        sensor = ObstacleSensor()
        sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
        env.add_sensor(sensor)
        ob, _, _, _ = env.step(np.zeros(env.n()))


        brain = None
        if not user_input_mode:
            brain = RBrain()
            brain.setup({
                "dt": DT,
                "robot_type": robot_type,
                "objects_in_env": True,
                "default_action": np.zeros(2),
                "objects": swap,
                "task": [("small_box", State(pos=np.array([2, -1, 0]))),
                        ("small_cylinder", State(pos=np.array([2, 3, 0])))],
                "env": env,
                "n_env": i,
                "kgraph": kgraph,
                "save_path": save_path,
            }, ob)

        try:
            for _ in range(10000):

                if user_input_mode:
                    conn.send({"request_action": True, "kill_child": False, "ob": ob})
                    keyboard_data = conn.recv()
                    action = keyboard_data["action"]
                    ob, _, _, _ = env.step(action)
                else:
                    action[0:2] = brain.respond()
                    ob, _, _, _ = env.step(action)
                    brain.update(ob)

        except StopIteration as exc:

            print(f"Tear down this environment, we're done here because {exc}")
            continue

        print('times is up, try again')

        if CREATE_SERVER_DASHBOARD:
            stop_dash_server(brain.dash_app)

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
