import numpy as np
import torch
from robot_brain.state import State
from robot_brain.controller.mppi.mppi_2th_order import Mppi2thOrder
from robot_brain.controller.mppi.mppi_3th_order import Mppi3thOrder
from robot_brain.controller.mppi.mppi_4th_order import Mppi4thOrder
from robot_brain.controller.mppi.mppi_6th_order import Mppi6thOrder

import numpy as np
import gym
import urdfenvs.point_robot_urdf # pylint: disable=unused-import
import urdfenvs.boxer_robot # pylint: disable=unused-import
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from robot_brain.state import State
from robot_brain.global_variables import DT


def main():
    """ MPPI test for the gym environment. """

    # robot_type = "pointRobot-vel-v7"
    # robot_type = "pointRobot-acc-v7"
    robot_type = "boxerRobot-vel-v7"
    # robot_type = "boxerRobot-acc-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())

    pos0 = np.array([1.0, 1.0])
    vel0 = np.array([0.0, 0.0])
    ob = env.reset(pos=pos0, vel=vel0)

    n_steps = 10000


    # mppi_controller = Mppi4thOrder()
    # def dynamics(x, u):
    #
    #     x_next = torch.zeros(x.shape, dtype=torch.float64, device=torch.device("cpu"))
    #
    #     x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT) # x_next[0] = x[0] + DT*u[0]
    #     x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT) # x_next[1] = x[1] + DT*u[1]
    #
    #     return x_next

    mppi_controller = Mppi3thOrder()
    def dynamics(x, u):
            
        x_next = torch.zeros(x.shape, dtype=torch.float64, device=torch.device("cpu"))

        x_next[:,0] = x[:,0] + DT*torch.cos(x[:,2])*u[:,0] 
        x_next[:,1] = x[:,1] + DT*torch.sin(x[:,2])*u[:,0] 
        x_next[:,2] = x[:,2] + DT*u[:,1]

        return x_next

    # mppi_controller = Mppi4thOrder()
    # def dynamics(x, u):
    #         
    #     x_next = torch.zeros(x.shape, dtype=torch.float64, device=torch.device("cpu"))
    #
    #     x_next[:,0] = x[:,0] + 1*DT*x[:,2] #+ 0.05*DT*u[:,0]*u[:,0] # x_pos_next = x_pos + DT * x_vel
    #     x_next[:,1] = x[:,1] + 1*DT*x[:,3] #+ 0.05*DT*u[:,0]*u[:,1] # y_pos_next = y_pos + DT * y_vel
    #     x_next[:,2] = x[:,2] + 1*DT*u[:,0] # x_vel_next = x_vel + DT * acc_x
    #     x_next[:,3] = x[:,3] + 1*DT*u[:,1] # y_vel_next = y_vel + DT * acc_y
    #
    #     return x_next
    
    # mppi_controller = Mppi6thOrder()
    # def dynamics(x, u):
    #         
    #     x_next = torch.zeros(x.shape, dtype=torch.float64, device=torch.device("cpu"))
    #
    #     x_next[:,0] = x[:,0] + 1*DT*x[:,3] #+ 0.05*DT*u[:,0]*u[:,0] # x_pos_next = x_pos + DT * x_vel
    #     x_next[:,1] = x[:,1] + 1*DT*x[:,4] #+ 0.05*DT*u[:,0]*u[:,1] # y_pos_next = y_pos + DT * y_vel
    #     x_next[:,2] = x[:,2] + 1*DT*x[:,5] # x_vel_next = x_vel + DT * acc_x
    #     x_next[:,3] = x[:,3] + 1*DT*torch.cos(u[:,0]) # y_vel_next = y_vel + DT * acc_y
    #     x_next[:,4] = x[:,4] + 1*DT*torch.sin(u[:,0]) # x_vel_next = x_vel + DT * acc_x
    #     x_next[:,5] = x[:,5] + 1*DT*u[:,1] # y_vel_next = y_vel + DT * acc_y
    #
    #     return x_next


    mppi_controller.setup(dyn_model=dynamics,
           current_state=State(),
           target_state=State(pos=np.array([2,1,0])))

    # add sensors
    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)


    # assuming you have a gym-like env
    for i in range(n_steps):

        if i == 50:
            mppi_controller.visualise(save=False)

        current_state = State(pos=ob["joint_state"]["position"])
        action[0:2] = mppi_controller.respond(current_state=current_state)

        # print(f'action: {action}')

        ob, reward, done, _ = env.step(action)
        # print(f"Observation: {ob}")


if __name__ == "__main__":
    main()
