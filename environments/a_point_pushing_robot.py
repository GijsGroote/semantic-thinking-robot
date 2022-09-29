from multiprocessing import Process, Pipe
import numpy as np
import gym
import urdfenvs.point_robot_urdf
import urdfenvs.boxer_robot
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from robot_brain.rbrain import RBrain
from robot_brain.planning.state import State
from robot_brain.controller.mpc.mpc import Mpc
from robot_brain.global_variables import DT


from environments.objects.boxes import box
from environments.objects.spheres import sphere
from environments.objects.cylinders import cylinder


user_input_mode = False

def main(conn=None):
    """
    Point robot and obstacles which can interact with each other in the environment.

    Semantic brain goal: find out how interachtin with the objects goes

    """
    # env = gym.make('point-robot-urdf-vel-v0', dt=dt, render=True)
    env = gym.make('boxer-robot-vel-v0', dt=DT, render=True)

    pos0 = np.array([1.0, 0.1])
    vel0 = np.array([0.0, 0.0])
    env.reset(pos=pos0, vel=vel0)
    default_action = np.array([0.0, 0.0])
    n_steps = 10000

    obstacles = {box.name(): box,
            sphere.name(): sphere,
            cylinder.name(): cylinder}

    # add obstacles
    env.add_obstacle(box)
    env.add_obstacle(sphere)
    env.add_obstacle(cylinder)

    # add sensors
    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)

    ob, reward, done, info = env.step(default_action)

    brain = RBrain()
    brain.setup({
        "dt": DT,
        "default_action": default_action,
        "target_state": State(pos=np.array([0, 0, 0])),
        "obstacles": obstacles
    }, ob)

    action =default_action

    for i in range(n_steps):
        if i == 500:
            brain.controller.set_target_state(State(pos=np.array([2,3,0])))
        if i == 700:
            brain.controller.set_target_state(State(pos=np.array([-2,3,2])))
        if i == 900:
            brain.controller.set_target_state(State(pos=np.array([2,-3,1])))

        if user_input_mode:
            conn.send({"request_action": True, "kill_child": False, "ob": ob})
            keyboard_data = conn.recv()
            action[0:2] = keyboard_data["action"]
        else:
            action = brain.respond()


        ob, reward, done, info = env.step(action)
        print(ob["joint_state"]["position"])

        brain.update(ob)


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

        responder.setup()
        # responder.setup(custom_on_press=custom_on_press)

        # start child process which keeps responding/looping
        responder.start(p)

        # kill parent process
        p.kill()
