import math
import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import

from motion_planning_env.box_obstacle import BoxObstacle
# from motion_planning_env.cylinder_obstacle import CylinderObstacle

from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import CREATE_SERVER_DASHBOARD, DT
from dashboard.app import stop_dash_server
from robot_brain.global_planning.kgraph.kgraph import KGraph

from helper_functions.figures import create_new_directory


blockade_obstacles = {
        "red_box": BoxObstacle(name="red_box", content_dict={
            "movable": True,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [255, 0, 0],
            "position": [-0.7, 0, 0.4],
            "geometry": {"length": 0.8, "width": 0.8, "height": 0.8},
            }),
        "white_wall_1": BoxObstacle(name="white_wall_1", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, math.pi/2],
            "color": [220,220,220,1],
            "position": [1.3, -1.9, 0.25],
            "geometry": {"length": 3, "width": 3, "height": 0.5},
            }),
        "white_wall_2": BoxObstacle(name="white_wall_2", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, math.pi/2],
            "color": [198,198,198,1],
            "position": [-1.7, -1.2, 0.5],
            "geometry": {"length": 1.6, "width": 0.3, "height": 1},
            }),
        "white_wall_3": BoxObstacle(name="white_wall_3", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, math.pi/2],
            "color": [198,198,198,1],
            "position": [-1.0, -3, 0.5],
            "geometry": {"length": 2, "width": 2, "height": 1},
            }),
        # "target_cylinder": CylinderObstacle(name="target_cylinder", content_dict={
        #     "movable": False,
        #     "type": "cylinder",
        #     "orientation": [0, 0, 0],
        #     "color": [0,255,0,1],
        #     "position": [-10.6, -50, 0.15],
        #     "geometry": {"radius": 0.3, "height": 0.3},
        #     }),
        }


def main():
    """
    Mimick the environment from paper:

    Affordance-Based Mobile Robot Navigation Among Movable Obstacles
    DOI: 10.1109/IROS45743.2020.9341337

    """
    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)
    kgraph = KGraph()

    # create new directory for data
    save_path = create_new_directory(dir_path="environments/mimick/mimick_wang/data")

    # try to solve the blockade task multiple times
    for i in range(100):

        print(f'starting blockade environment: {i}')

        action = np.zeros(env.n())
        ob = env.reset()
        env.add_obstacle(blockade_obstacles["red_box"])
        env.add_obstacle(blockade_obstacles["white_wall_1"])
        env.add_obstacle(blockade_obstacles["white_wall_2"])
        env.add_obstacle(blockade_obstacles["white_wall_3"])
        # env.add_obstacle(blockade_obstacles["target_cylinder"])


        sensor = ObstacleSensor()
        sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
        env.add_sensor(sensor)


        # env.add_target_ghost(
        #     "target_cylinder",
        #     [-0.7, -0.7, 0]
        #     )

        ob, reward, done, info = env.step(np.zeros(env.n()))
        brain = RBrain()

        brain.setup({
            "dt": DT,
            "robot_type": robot_type,
            "objects_in_env": True,
            "default_action": np.zeros(2),
            "task": [
                ("robot", State(pos=np.array([-0.7, -0.7, 0]))),
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
            kgraph.visualise(save=False)
            kgraph = KGraph()
            continue

        print('times is up, try again')

        kgraph.visualise(save=False)
        kgraph = KGraph()

        if CREATE_SERVER_DASHBOARD:
            stop_dash_server(brain.dash_app)

if __name__ == "__main__":
    main()

