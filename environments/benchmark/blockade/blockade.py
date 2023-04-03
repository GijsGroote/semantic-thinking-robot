import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import CREATE_SERVER_DASHBOARD, DT
from dashboard.app import stop_dash_server
from robot_brain.global_planning.kgraph.kgraph import KGraph

from environments.benchmark.benchmark_obstacles.obstacles import blockade_obstacles

def main():
    """
    Point robot which can drive around in its environment using a mpc controller.
    """
    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)
    kgraph = KGraph()

    # try to solve the blockade task multiple times
    for i in range(8):
        print(f'starting blockade environment: {i}')

        action = np.zeros(env.n())
        ob = env.reset()
        env.add_obstacle(blockade_obstacles["simpleCylinder"])
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
                (blockade_obstacles["simpleBox"].name(), State(pos=np.array([3, 0, 0]))),
                ],
            "objects": blockade_obstacles,
            "env": env,
            "n_env": i,
            "kgraph": kgraph,
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
