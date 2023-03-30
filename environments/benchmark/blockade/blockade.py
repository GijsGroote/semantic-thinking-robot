import urdfenvs.boxer_robot
import urdfenvs.point_robot_urdf # pylint: disable=unused-import

from multiprocessing import Process, Pipe
import numpy as np
import gym
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State

from robot_brain.global_variables import DT

from environments.benchmark.benchmark_obstacles.obstacles import blockade_obstacles

def main():
    """
    Point robot which can drive around in its environment using a mpc controller.
    """
    env = gym.make("pointRobot-vel-v7", dt=DT, render=True)
    ob = env.reset()
    action = np.zeros(env.n())

    # env.add_obstacle(blockade_obstacles["urdf_duck"])
    env.add_obstacle(blockade_obstacles["simpleCylinder"])
    env.add_obstacle(blockade_obstacles["simpleBox"])
    env.add_obstacle(blockade_obstacles["wall1"])
    env.add_obstacle(blockade_obstacles["wall2"])
    env.add_obstacle(blockade_obstacles["wall3"])

    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)

    ob, reward, done, info = env.step(np.zeros(env.n()))

    for i in range(50):
        ob, _, _, _ = env.step(action)


    box_target_state = State(pos=np.array([3, 0, 0]))
    # print("now adding the robot")
    # env.add_robot_ghost("boxerRobot-vel-v7", box_target_state.get_2d_pose(), 0.2)
    # print("now done adding the robot")

    env.add_target_ghost(blockade_obstacles["simpleBox"].name(), box_target_state.get_2d_pose())

    brain = RBrain()
    brain.setup({
        "dt": DT,
        "robot_type": "boxer_robot",
        "obstacles_in_env": True,
        "obstacles": blockade_obstacles,
        "task": [
                (blockade_obstacles["simpleBox"].name(), box_target_state),
                ]
    }, ob)

    for i in range(10000):

        if i % 50 == 0:

            brain.plot_occupancy_graph()
        
        if i==200:
            brain.controller.set_target_state(State(pos=np.array([1,3,0])))

        action = brain.respond()
        ob, _, _, _ = env.step(action)
        brain.update(ob)

if __name__ == "__main__":

    main()

#     else:
#         # setup multi threading with a pipe connection
#         parent_conn, child_conn = Pipe()
#
#         # create parent process
#         p = Process(target=main, args=(parent_conn,))
#         # start parent process
#         p.start()
#
#         # create Responder object
#         responder = Responder(child_conn)
#
#         # logical key bindings
#         custom_on_press = {Key.down: np.array([-3.0, 0.0]),
#                            Key.up: np.array([3.0, 0.0]),
#                            Key.left: np.array([0.0, 3.0]),
#                            Key.right: np.array([0.0, -3.0])}
#
#         # responder.setup(defaultAction=np.array([0.0, 0.0]))
#         responder.setup(custom_on_press=custom_on_press)
#
#         # start child process which keeps responding/looping
#         responder.start(p)
#
#         # kill parent process
#         p.kill()
