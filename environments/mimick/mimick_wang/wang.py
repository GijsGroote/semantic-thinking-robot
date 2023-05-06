import numpy as np
import math
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import


from motion_planning_env.urdf_obstacle import UrdfObstacle
from motion_planning_env.box_obstacle import BoxObstacle

from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import CREATE_SERVER_DASHBOARD, DT
from dashboard.app import stop_dash_server
from robot_brain.global_planning.kgraph.kgraph import KGraph

from helper_functions.figures import create_time_plot, create_prediction_error_plot, create_new_directory


blockade_obstacles = {
        "simpleBox": BoxObstacle(name="simpleBox", content_dict={
            "movable": True,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [22, 63, 88],
            "position": [1, -1, 0.7],
            "geometry": {"length": 1.4, "width": 1.4, "height": 1.4},
            }),
        "wall1": BoxObstacle(name="wall1", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, math.pi/2],
            "color": [245, 181, 27, 1],
            "position": [4.1, -1, 0.2],
            "geometry": {"length": 2.7, "width": 0.3, "height": 0.4},
            }),
        "wall2": BoxObstacle(name="wall2", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,0],
            "color": [245, 181, 27, 1],
            "position":[3, 0.2, 0.2],
            "geometry": {"length": 2, "width": 0.3, "height": 0.4},
            }),
        "wall3": BoxObstacle(name="wall3", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,0],
            "color": [245, 181, 27, 1],
            "position": [3, -2.2, 0.2],
            "geometry": {"length": 2, "width": 0.3, "height": 0.4},
            },
            ),
        }


def main():
    """
    Point robot which can drive around in its environment using a mpc controller.
    """
    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)
    kgraph = KGraph()

    # create new directory for data
    save_path = create_new_directory(dir_path="logger/logs")

    # try to solve the blockade task multiple times
    for i in range(3):

        print(f'starting blockade environment: {i}')

        action = np.zeros(env.n())
        ob = env.reset()
        env.add_obstacle(blockade_obstacles["simpleBox"])
        env.add_obstacle(blockade_obstacles["wall1"])
        env.add_obstacle(blockade_obstacles["wall2"])
        env.add_obstacle(blockade_obstacles["wall3"])

        sensor = ObstacleSensor()
        sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
        env.add_sensor(sensor)

        ob, reward, done, info = env.step(np.zeros(env.n()))
        brain = RBrain()

        brain.setup({
            "dt": DT,
            "robot_type": robot_type,
            "objects_in_env": True,
            "default_action": np.zeros(2),
            "task": [
                ("robot", State(pos=np.array([3, -1, 0]))),
                ],
            "objects": blockade_obstacles,
            "env": env,
            "n_env": i,
            "kgraph": kgraph,
            "save_path": save_path,
            }, ob)

        try:
            for _ in range(10000):

                ob, _, _, _ = env.step(action)

                action[0:2] = brain.respond()
                ob, _, _, _ = env.step(action)
                brain.update(ob)

        except StopIteration as exc:

            print(f"Tear down this environment, we're done here because {exc}")
            continue

        print('times is up, try again')

        if CREATE_SERVER_DASHBOARD:
            stop_dash_server(brain.dash_app)

if __name__ == "__main__":
    main()

