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
from robot_brain.controller.push.mppi.mppi_4th_order import PushMppi4thOrder
from robot_brain.system_model import SystemModel
from environments.mppi_push.obstacles import box
from helper_functions.geometrics import which_side_point_to_line



def main():
    robot_type = "pointRobot-vel-v7"
    # robot_type = "boxerRobot-vel-v7"
    env = gym.make(robot_type, dt=DT, render=True)

    action = np.zeros(env.n())
    env.reset()


    env.add_obstacle(box)
    sensor = ObstacleSensor()
    sensor.set_bullet_id_to_obst(env.get_bullet_id_to_obst())
    env.add_sensor(sensor)
    ob, reward, done, info = env.step(action)

    # test controller
    controller = PushMppi4thOrder()
    def model(x, u):
        # this model describes the robot and objects as a solid block. they move as if stuck together

        x_next = torch.zeros(x.shape, dtype=torch.float64, device=TORCH_DEVICE)
        
        x_next[:,0] = torch.add(x[:,0], u[:,0], alpha=DT)
        x_next[:,1] = torch.add(x[:,1], u[:,1], alpha=DT)
        x_next[:,2] = torch.add(x[:,2], u[:,0], alpha=DT)
        x_next[:,3] = torch.add(x[:,3], u[:,1], alpha=DT)

        return x_next

    dyn_model = SystemModel(model, 'mppi_test_model')

    box_target = State(pos=np.array([-3, 0, 0]), ang_p=np.array([0, 0, 0]))
    env.add_target_ghost(box.name(), box_target.get_2d_pose())

    # box_targets = []
    # target_selector = 0
    # for i in range(50):
    #     box_targets.append(State(pos=np.array([-i/2-2, -(i/3), 0]), ang_p=np.array([0, 0, i*math.pi/30])))

    controller.setup(dyn_model,
            State(pos=ob['joint_state']['position']),
            State(pos=ob['obstacleSensor'][box.name()]['pose']['position']),
            box_target)


    for i in range(1, 10000):

        # if i % 25 == 0:
        #     controller.set_target_state(box_targets[target_selector])
        #     env.add_target_ghost(box.name(), box_targets[target_selector].get_2d_pose())
        #     target_selector += 1


        if i > 50:
            action[0:2] = controller.respond(robot_state, obstacle_state)


        robot_state = State(pos=ob['joint_state']['position'])
        obstacle_state = State(pos=ob['obstacleSensor'][box.name()]['pose']['position'],
            ang_p= ob['obstacleSensor'][box.name()]['pose']['orientation'])


        ob, reward, done, info = env.step(action)

if __name__ == "__main__":
    main()
