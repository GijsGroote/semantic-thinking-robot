import numpy as np
import torch
import gym
import math
import time 
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
# import urdfenvs.boxer_robot # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
# from robot_brain.rbrain import RBrain
from robot_brain.state import State
from robot_brain.global_variables import DT, TORCH_DEVICE
from robot_brain.controller.push.mppi.mppi_5th_order import PushMppi5thOrder 
from environments.mppi_push.obstacles import box
from helper_functions.geometrics import which_side_point_to_line 


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

        # width square obstacle
        H = 2

        # point a in the center of the pushed against edge
        xa = x[:,2]+torch.sin(x[:,4])*H/2
        ya = x[:,3]-torch.cos(x[:,4])*H/2

        # line parallel to the obstacle edge being pushed against
        a_abc = torch.tan(math.pi/2-x[:,4])
        b_abc = x[:,2]+H/2*torch.sin(x[:,4])-torch.tan(math.pi/2-x[:,4])*(x[:,3]-H/2*torch.cos(x[:,4]))


        # line from center robot to center obstacle
        a_ro = (x[:,0]-x[:,2])/(x[:,1]-x[:,3])
        b_ro = x[:,2]-(x[:,0]-x[:,2])/(x[:,1]-x[:,3])*x[:,3]

        yb = (b_ro-b_abc)/(a_abc-a_ro)
        xb = a_abc*yb+b_abc

        # st is the distance from pushing point to the centerline of the obstacle perpendicular to the edge which is pushed against
        st=1.2*torch.sqrt((xa-xb)**2+(ya-yb)**2)

        # obstacle rotating clockwise (positive) or anti-clockwise (negative)
        st = which_side_point_to_line(
                x[:,2:4],
                torch.stack((xa, ya), dim=-1),
                torch.stack((xb, yb), dim=-1))*st

        # velocity of robot perpendicular to box at point p
        vp = u[:,0]*torch.sin(x[:,4]) + u[:,1]*torch.cos(x[:,4])


        x_next = torch.zeros(x.shape, dtype=torch.float64, device=TORCH_DEVICE)
        x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
        x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]
        x_next[:,2] = torch.add(x[:,2], torch.sin(x[:,4])*(1-torch.abs(2*st/H))*vp, alpha=DT)
        x_next[:,3] = torch.add(x[:,3], torch.cos(x[:,4])*(1-torch.abs(2*st/H))*vp, alpha=DT)
        x_next[:,4] = torch.add(x[:,4], 2*vp/H*st, alpha=DT)

        return x_next

    box_target = State(pos=np.array([-1.7, 0, 0]), ang_p=np.array([0, 0, 0.2]))
    box_targets = []
    target_selector = 0
    for i in range(50):
        box_targets.append(State(pos=np.array([-i/2-2, -(i/3), 0]), ang_p=np.array([0, 0, i*math.pi/30])))

    controller.setup(dyn_model,
            State(pos=ob['joint_state']['position']),
            State(pos=ob['obstacleSensor'][box.name()]['pose']['position']),
            box_target)

            
    for i in range(1, n_steps):

        if i % 25 == 0: 
            controller.set_target_state(box_targets[target_selector])
            env.add_target_ghost(box.name(), box_targets[target_selector].get_2d_pose())
            target_selector += 1

        
        if i > 50:

            action[0:2] = controller.respond(robot_state, obstacle_state)

        
        robot_state = State(pos=ob['joint_state']['position'])
        obstacle_state = State(pos=ob['obstacleSensor'][box.name()]['pose']['position'], 
            ang_p= ob['obstacleSensor'][box.name()]['pose']['orientation'])

        
        # action[0:2] = controller._find_input(robot_state, obstacle_state)

        ob, reward, done, info = env.step(action)

if __name__ == "__main__":
    main()
