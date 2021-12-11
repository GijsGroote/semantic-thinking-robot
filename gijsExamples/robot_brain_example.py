import numpy as np
from robot_brain.RBrain import RBrain
from robot_brain.State import State

user_input_mode = False


def main():


    brain = RBrain()

    pos0 = np.array([1.0, 2.0])
    vel0 = np.array([1.0, 3.0])

    stat_world_info = {"robot":
        {
            "pos": pos0,
            "vel": vel0,
        }
    }
    brain.setup(stat_world_info)

    targetState = State()   # drive to (0,0,0,0,0)
    brain.set_OF(brain.robot, targetState)

    print(brain.calculate_OF())

if __name__ == '__main__':
    main()

