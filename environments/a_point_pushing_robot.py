import math
import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT
from robot_brain.global_planning.kgraph.kgraph import KGraph

from environments.objects.boxes import box, box2
from environments.objects.cylinders import cylinder


def main():
    """
    Point robot and objects which can interact with each other in the environment.

    Semantic brain goal: find out how interachtin with the objects goes

    """
    robot_type = "pointRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)
    kgraph = KGraph()

    for i in range(10):
        print(f'starting blockade environment: {i}')

        action = np.zeros(env.n())
        env.reset()

        objects = {box.name(): box,
                box2.name(): box2,
                cylinder.name(): cylinder}

        # add objects
        env.add_obstacle(box)
        env.add_obstacle(box2)
        env.add_obstacle(cylinder)

        # add sensors
        sensor = ObstacleSensor()
        sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
        env.add_sensor(sensor)

        ob, reward, done, info = env.step(action)

        brain = RBrain()
        brain.setup({
            "dt": DT,
            "robot_type": robot_type,
            "objects_in_env": True,
            "default_action": np.array(np.zeros(2)),
            "task": [
                (box.name(), State(pos=np.array([2, -3.31, 0.1]))),
                # (box2.name(), State(pos=np.array([-3, 1.31, 0.1]))),
                (cylinder.name(), State(pos=np.array([-4, -2.31, 0.1]))),
                # ("robot", State(pos=np.array([-4.3212, -2.9, 0]))),
                # ("robot", State(pos=np.array([3.3212, -2, -math.pi/2]))),
                # ("robot", State(pos=np.array([-0.3212, .1, 0]))),
                # ("robot", State(pos=np.array([3.3212, 2.20, 0]))),
                # ("robot", State(pos=np.array([4,-2,0]))),
                # ("robot", State(pos=np.array([-4, -4, 0]))),
                ],
            "objects": objects,
            "env": env,
            "n_env": i,
            "kgraph": kgraph,
        }, ob)

        brain.update(ob)

        try:
            for _ in range(10000):

                action[0:2] = brain.respond()
                ob, _, _, _ = env.step(action)
                brain.update(ob)

        except StopIteration as exc:

            print(f"Tear down this environment, we're done here because {exc}")
            continue

        print('times is up, try again')


if __name__ == "__main__":
    main()
