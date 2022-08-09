import numpy as np
import gym
import urdfenvs.boxer_robot
from pynput.keyboard import Key
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from multiprocessing import Process, Pipe
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.RBrain import RBrain
from robot_brain.RBrain import State

from environments.objects.urdf_objects import urdf_duck2
from environments.objects.walls import surrounded 

target_pos = np.array([0, 0, 0])
target_ang_p = np.array([0, 0, 0])

user_input_mode = False

def main(conn=None):
    dt = 0.05
    env = gym.make('boxer-robot-vel-v0', dt=dt, render=True)

    n_steps = 1000
    ob = env.reset()
    
    # this should be done much easiers
    env.add_walls(dim=surrounded["wall2"]["dim"], poses_2d=surrounded["wall2"]["poses_2d"])
    env.add_walls(dim=surrounded["wall1"]["dim"], poses_2d=surrounded["wall1"]["poses_2d"])
    env.add_walls(dim=surrounded["wall3"]["dim"], poses_2d=surrounded["wall3"]["poses_2d"])
    env.add_walls(dim=surrounded["wall4"]["dim"], poses_2d=surrounded["wall4"]["poses_2d"])
    env.add_walls(dim=surrounded["wall5"]["dim"], poses_2d=surrounded["wall5"]["poses_2d"])
    env.add_walls(dim=surrounded["wall6"]["dim"], poses_2d=surrounded["wall6"]["poses_2d"])

    env.add_obstacle(urdf_duck2) 
    ob, _, _, _ = env.step(np.array([0, 0]))

    brain = None
    if not user_input_mode:
        sensor = ObstacleSensor()
        env.add_sensor(sensor)
        targetState = State(pos=target_pos, ang_p=target_ang_p)
        brain = RBrain()
        brain.setup({
            "dt": dt,
            "targetState": targetState,
            "obstacles": []
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
            print(ob)
            brain.update(ob)



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
                           Key.left: np.array([0.0, 1.0]),
                           Key.right: np.array([0.0, -1.0])}

        # responder.setup(defaultAction=np.array([0.0, 0.0]))
        responder.setup(custom_on_press=custom_on_press)

        # start child process which keeps responding/looping
        responder.start(p)

        # kill parent process
        p.kill()
