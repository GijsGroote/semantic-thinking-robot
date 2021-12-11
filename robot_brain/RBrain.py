from robot_brain.State import State
from robot_brain.Dynamics import Dynamics
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

    def __init__(self, conn=None):
        self.conn = conn  # Connection to the world the robot lives in
        self.objects = []  # Object information list
        self.verbs = []   # verbs which can be used
        self.robot = None  # Player information
        self.map = None  # Map used for planning
        self.is_doing = IS_DOING_NOTHING  # State indicating what the brain is doing

        self.height_of_map = 0  # Map height
        self.width_of_map = 0  # Map width
        self.robot_accepts_input = False  # boolean indicting if robot input is accepted
        self.of = None  # objective function
        self.action = None

    def setup(self, stat_world_info):

        # robot and objects
        r_info = stat_world_info["robot"]
        p = r_info["pos"]
        v = r_info["vel"]
        s0 = State(pos_x=p[0], pos_y=p[1], vel_x=v[0], vel_y=p[1])
        robot = Object(s0, 1, 1)
        self.robot = robot
        self.objects.append(robot)
        self.action = np.array([0.0, 0.0])

        # create dynamics
        d1 = Dynamics()

        


    def respond(self):
        """ Respond to request with the latest action """
        # receive request
        request = self.conn.recv()
        if request["request_action"] and self.is_doing is IS_EXECUTING:
            # send action
            self.conn.send({
                "RBState": self.is_doing,
                "action": self.action
            })
        if request["request_action"] and self.is_doing is IS_THINKING:
            # send action
            self.conn.send({
                "RBState": self.is_doing,
                "action": self.action
            })

        if request["request_action"] and self.is_doing is IS_DOING_NOTHING:

            # create thinking three, use everything you've got to come up with a valid plan
            # think of mpc plan


            self.conn.send({
                "RBState": self.is_doing,
                "action": self.action
            })
        elif request["kill_child"]:
            raise Exception
        else:
            raise Exception("Cannot handle action request")

        
    def start(self, p):
        """
        start with responding to keyboard input

        :param p: Parent process
        """

        # while parent process is alive, keep on responding
        while p.is_alive():
            try:
                # think of a plan to reach target, inform world that we are thinking about something
                self.respond()
            except Exception:
                return

    def calculate_plan(self):
        # set brain state to thinking
        self.is_doing = IS_THINKING

        # send zero input and update world what the robot is doing
        self.conn.send({"RBState": self.is_doing, "x": 0, "y": 0})

        print("Can I think of something...")

        # wait 2 seconds
        time.sleep(2)

        print("yes I got it! executing plan")
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





