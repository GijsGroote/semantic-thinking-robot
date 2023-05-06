import numpy as np
import gym
import urdfenvs.boxer_robot
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.rbrain import RBrain
from robot_brain.state import State

from environments.objects.boxes import box
from environments.objects.spheres import sphere
from environments.objects.cylinders import cylinder

target_pos = np.array([3, 2, 0])
target_ang_p = np.array([0, 0, 2])
controller = "mpc"


def main():
    """
    Point robot which can drive around in its environment using a mpc controller.
    """
    dt = 0.05
    # env = gym.make("boxerRobot-vel-v7", dt=dt, render=True)
    env = gym.make("pointRobot-vel-v7", dt=dt, render=True)

    default_action = np.array([0, 0, 0.0])
    n_steps = 1000
    ob = env.reset()
    # sensor = ObstacleSensor()
    # env.add_sensor(sensor)
    # update these walls
    # env.add_walls()

    env.add_obstacle(cylinder)

    ob, reward, done, info = env.step(default_action)

    target_state = State(pos=target_pos, ang_p=target_ang_p)
    # brain = RBrain()
    # # do the regular stuff, like begin the simulation, something like that
    # brain.setup({
    #     "dt": dt,
    #     "default_action": default_action,
    #     "target_state": target_state,
    #     "controller": controller,
    #     "additionalParameters??": True
    # }, ob)

    action =default_action 

    for i in range(n_steps):

        # action = brain.respond()
        action = default_action

        ob, reward, done, info = env.step(action)
         
        # brain.update(ob)


if __name__ == '__main__':
    main()
