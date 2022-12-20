import numpy as np
import torch
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
# import urdfenvs.boxer_robot # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
# from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT

from robot_brain.controller.push.mppi.mppi_2th_order import PushMppi2thOrder 

from environments.objects.boxes import box, box2


def main():
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

    obstacles = {box.name(): box, box2.name(): box2}

    # add obstacles
    env.add_obstacle(box)
    env.add_obstacle(box2)

    # add sensors
    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)

    ob, reward, done, info = env.step(action)

    robot_state = State(pos=ob['joint_state']['position'])
    obstacle_state = State(pos=ob['obstacleSensor']['simple_box']['pose']['position'], 
            ang_p= ob['obstacleSensor']['simple_box']['pose']['orientation'])

    controller = PushMppi2thOrder()

    def dyn_model(x, u):
        x_next = torch.zeros(x.shape, dtype=torch.float64, device=torch.device("cpu"))
        x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
        x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]
        return x_next

    controller.setup(dyn_model, robot_state, obstacle_state, State(pos=np.array([1,1,0])))

    for i in range(n_steps):


        robot_state = State(pos=ob['joint_state']['position'])
        obstacle_state = State(pos=ob['obstacleSensor']['simple_box']['pose']['position'], 
            ang_p= ob['obstacleSensor']['simple_box']['pose']['orientation'])
        
        action[0:2] = controller._find_input(robot_state, obstacle_state)
        ob, reward, done, info = env.step(action)

if __name__ == "__main__":
    main()
