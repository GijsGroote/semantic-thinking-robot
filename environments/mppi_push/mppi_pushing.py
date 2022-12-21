import numpy as np
import torch
import gym
import math
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
# import urdfenvs.boxer_robot # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
# from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT
from robot_brain.controller.push.mppi.mppi_5th_order import PushMppi5thOrder 
from environments.mppi_push.obstacles import box

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

    # add obstacles
    env.add_obstacle(box)

    # add sensors
    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)

    ob, reward, done, info = env.step(action)

    controller = PushMppi5thOrder()

    def dyn_model(x, u):
        vp = u[:,0]*torch.sin(x[:,4]) + u[:,1]*torch.sin(x[:,4])
        # todo, that st thingy
        # st = torch.sqrt(()+())
        st=0.1
        x_next = torch.zeros(x.shape, dtype=torch.float64, device=torch.device("cpu"))

        x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
        x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]
        x_next[:,2] = torch.add(x[:,2], torch.sin(x[:,4])*vp, alpha=DT)
        x_next[:,3] = torch.add(x[:,3], torch.cos(x[:,4])*vp, alpha=DT)
        x_next[:,4] = torch.add(x[:,4], vp, alpha=DT)

        return x_next

    controller.setup(dyn_model,
            State(pos=ob['joint_state']['position']),
            State(pos=ob['obstacleSensor'][box.name()]['pose']['position']),
            State(pos=np.array([-10,-1,0])))

    for _ in range(n_steps):


        robot_state = State(pos=ob['joint_state']['position'])
        obstacle_state = State(pos=ob['obstacleSensor'][box.name()]['pose']['position'], 
            ang_p= ob['obstacleSensor'][box.name()]['pose']['orientation'])
        
        # action[0:2] = controller._find_input(robot_state, obstacle_state)
        action[0:2] = controller.respond(robot_state, obstacle_state)

        ob, reward, done, info = env.step(action)

if __name__ == "__main__":
    main()
