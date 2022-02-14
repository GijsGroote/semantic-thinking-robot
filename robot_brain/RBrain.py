from robot_brain.State import State
from robot_brain.Dynamics import Dynamics
from robot_brain.Controller import Controller
from robot_brain.Object import Object
import numpy as np
import time

# is_doing states
IS_DOING_NOTHING = "nothing"
IS_THINKING = "thinking"
IS_EXECUTING = "executing"


class RBrain:
    """
    RBrain Class

    Finds best input for the robot in order to reach a given task
    by learning world properties and minimizing cost function of given task
    """

    def __init__(self):
        self.objects = []  # Object information list
        self.verbs = []   # verbs which can be used
        self.robot = None  # Player information
        self.map = None  # Map used for planning
        self.is_doing = IS_DOING_NOTHING  # State indicating what the brain is doing
        self.controller = None

        self.height_of_map = 0  # Map height
        self.width_of_map = 0  # Map width
        self.robot_accepts_input = False  # boolean indicting if robot input is accepted
        self.of = None  # objective function
        self.action = None
        self.dt = None

    def setup(self, stat_world_info):

        # robot and objects
        r_info = stat_world_info["robot"]
        p = r_info["pos"]
        v = r_info["vel"]
        s0 = State(pos_x=p[0], pos_y=p[1], ang_p=p[2], vel_x=v[0], vel_y=v[1], ang_v=v[2])
        robot = Object(s0)
        self.robot = robot
        self.objects.append(robot)
        self.action = np.array([0.0, 0.0])
        self.dt = stat_world_info["dt"]
        # todo: objects list

    def update(self, info):
        # robot and objects
        p = info["x"]
        v = info["xdot"]
        self.robot.state.update_p_and_v(p[0], p[1], p[2], v[0], v[1], v[2])
        # todo: is the robot in the object list also updated?

    def respond(self):
        """ Respond to request with the latest action """
        # receive request
        if self.is_doing is IS_EXECUTING:
            # send action
            if self.controller is not None:
                return self.controller.respond(self.robot.state.pos2arr())
            else:
                print('returning default action')
                return np.array([0.0, 0.0])

        if self.is_doing is IS_THINKING:
            # send action
            return self.action

        if self.is_doing is IS_DOING_NOTHING:

            self.calculate_plan()

            return self.action
        else:
            raise Exception("todo, this error mesasgeCannot handle action request")

        
    def calculate_plan(self):
        # set brain state to thinking
        print("Can I think of something...")
        self.is_doing = IS_THINKING


        print("yes I got it, MPC! executing plan")
        contr = Controller(self.dt)
        dyn_model = Dynamics()
        dyn_model.set_boxer_model()
        contr.create_mpc_controller(dyn_model, self.robot.state.pos2arr())
        self.controller = contr

        self.is_doing = IS_EXECUTING

    def calculate_input(self):
        # todo: use controller/world states and other thingies to calculate the best input
        print("calculating best input")


    def set_OF(self, obj, state):
        """
        Sets the objective function

        :param obj:
        :param state:

        """
        # check if obj can reach target state
        # if not, can it reach it using backtracking

        self.of = state.euclidean

    def calculate_OF(self):
        """
        Calculate the value of the objective function for the current state

        :return:
        """
        if self.of is not None:
            return self.of(self.robot.state)
        else:
            raise Exception("objective function was not set")





