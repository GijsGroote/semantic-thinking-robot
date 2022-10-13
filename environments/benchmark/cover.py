import numpy as np
import gym
import urdfenvs.boxer_robot
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from pynput.keyboard import Key
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from multiprocessing import Process, Pipe
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from environments.objects.urdf_objects import urdf_duck2

target_pos = np.array([-2, -1, 0])
target_ang_p = np.array([0, 0, -1])

user_input_mode = False 

def main(conn=None):
    dt = 0.05
    env = gym.make("boxerRobot-vel-v7", dt=dt, render=True)

    ob = env.reset()
    
    env.add_obstacle(urdf_duck2)
    # medium sized shapes
    env.add_shapes("GEOM_SPHERE", dim=8, mass=100, poses_2d=[[-5,4,0], [-9, 4, 0]])
    env.add_shapes("GEOM_CILINDER", dim=[2, 2], mass=-1, poses_2d=[[-20,10,0], [-15,9,0]])
    env.add_shapes("GEOM_CAPSULE", dim=[2, 2], mass=-1, poses_2d=[[-18,5,0], [-11,4,2]])

    # env.add_shapes("GEOM_CAPSULE", dim=[1, 1], mass=15, poses_2d=[[0,-10,0], [4,-10,2], [8,-10,2], [8,-6,2], [8,-2,2], [19,1,2]], place_height=10)

    # large ass obstacle in the backgound
    env.add_shapes("GEOM_BOX", dim=[5,10,5], mass=-1, poses_2d=[[-8, 10, 3.1415/2]])
    env.add_shapes("GEOM_CAPSULE", dim=[10, 10], mass=-1, poses_2d=[[-5,28,2]])
    env.add_shapes("GEOM_BOX", dim=[5, 5, 8], mass=150, poses_2d=[[-11,12, 0.3], [12,10,0.7]], place_height=10)

    env.add_shapes("GEOM_BOX", dim=[1,1,1], mass=15, poses_2d=[[-1,-2,0.4], [-2,-1,2], [-0,-4,0.4], [-5,18,2]])
    env.add_shapes("GEOM_SPHERE", dim=2, mass=15, poses_2d=[[0,-2,0], [1, -1, 0], [3, -1,0], [1, 1, 0]], place_height=3)# cubes from the sky
    env.add_shapes("GEOM_BOX", dim=[1,1,1], mass=15, poses_2d=[[-3,2,0.4], [-6,1,2], [-1,-2,0.4], [-5,0,2]], place_height=6)
    # spheres from the sky
    env.add_shapes("GEOM_SPHERE", dim=2, mass=15, poses_2d=[[-1,0,0], [-2, -3, 0], [-1,-2,0], [-1, 2, 0]], place_height=8)


    sensor = ObstacleSensor()
    env.add_sensor(sensor)
    ob, _, _, _ = env.step(np.array([0.0, 0.0]))

    brain = None
    if not user_input_mode:
        targetState = State(pos=target_pos, ang_p=target_ang_p)
        brain = RBrain()
        brain.setup({
            "dt": dt,
            "targetState": targetState,
            "obstacles": []
        }, ob)

    for i in range(100000):

        if i < 50 and i >20:
            if i % 30:
                # cubes from the sky
                env.add_shapes("GEOM_BOX", dim=[1,1,1], mass=15, poses_2d=[[-3,17,0.4], [-6,17,2], [-1,18,0.4], [-5,18,2]], place_height=50+i)
                # spheres from the sky
                env.add_shapes("GEOM_SPHERE", dim=2, mass=15, poses_2d=[[-8,18,0], [-5, 18, 0], [-9, 19,0], [-6, 19, 0]], place_height=40+i)# cubes from the sky
            if i % 30:
                env.add_shapes("GEOM_BOX", dim=[1,1,1], mass=15, poses_2d=[[-3,7,0.4], [-6,7,2], [-1,8,0.4], [-5,8,2]], place_height=1+i)
                # spheres from the sky
                env.add_shapes("GEOM_SPHERE", dim=2, mass=15, poses_2d=[[-8,8,0], [-5, 8, 0], [-9,9,0], [-6, 9, 0]], place_height=i)

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
