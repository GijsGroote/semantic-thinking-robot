import numpy as np
import torch
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
# import urdfenvs.boxer_robot # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
#from multiprocessing import Process, Pipe
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from multiprocessing import Process, Pipe
from pynput.keyboard import Key
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT, TORCH_DEVICE
from robot_brain.controller.push.mppi.mppi_5th_order import PushMppi5thOrder 
from environments.mppi_push.obstacles import box


user_input_mode = True

def main(conn=None):
 
    """
    Point robot and obstacles which can interact with each other in the environment.

    Semantic brain goal: find out how interachtin with the objects goes

    """
    robot_type = "pointRobot-vel-v7"
    # robot_type = "boxerRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())

    pos0 = np.array([1.0, 0.1])
    vel0 = np.array([0.0, 0.0])
    env.reset(pos=pos0, vel=vel0)

    n_steps = 10000

    # add obstacles
    env.add_obstacle(box)

    # add sensors
    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)

    ob, reward, done, info = env.step(action)

    controller = PushMppi5thOrder()

    def dyn_model(x, u):
        x_next = torch.zeros(x.shape, dtype=torch.float64, device=TORCH_DEVICE)
        x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
        x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]
        return x_next

    controller.setup(dyn_model,
            State(pos=ob['joint_state']['position']),
            State(pos=ob['obstacleSensor'][box.name()]['pose']['position']),
            State(pos=np.array([-10,-6,0])))

    for _ in range(n_steps):



        robot_state = State(pos=ob['joint_state']['position'])
        obstacle_state = State(pos=ob['obstacleSensor'][box.name()]['pose']['position'], 
            ang_p= ob['obstacleSensor'][box.name()]['pose']['orientation'])
        
        if user_input_mode:
            conn.send({"request_action": True, "kill_child": False, "ob": ob})
            keyboard_data = conn.recv()
            action[0:2] = keyboard_data["action"]
        else:

            action[0:2] = controller.respond(robot_state, obstacle_state)

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
