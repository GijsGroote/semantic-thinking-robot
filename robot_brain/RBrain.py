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

    def __init__(self, parent_conn):
        self.connection = parent_conn  # Connection to the world the robot lives in
        self.objects = None  # Object information list
        self.player = None  # Player information
        self.map = None  # Map used for planning
        self.is_doing = IS_DOING_NOTHING  # State indicating what the brain is doing

        self.height_of_map = 0  # TileMap height
        self.width_of_map = 0  # TileMap width
        self.brain = None  # controller of the player, User or Robot
        self.robot_accepts_input = False  # boolean indicting if robot input is accepted

    def setup(self, stat_world_info):
        print("setup the robot brain please")
        print(stat_world_info)

        # todo: create all objects for robot brain
        # create states
        s1 = State(10, 10)
        s2 = State(50, 50)

        # todo: create objects
        o1 = Object(s1, 5, 5)
        o2 = Object(s2, 5, 5)

        # create dynamics
        d1 = Dynamics()
        o1.dynamics = d1
        o2.dynamics = d1

        # create world
        # w = World(100, 140)
        # w.objects.append(o1)
        # w.objects.append(o2)
        #
        # for object in w.objects:
        #     print(object.state.vel_x)

    def calculate_plan(self):
        # set brain state to thinking
        self.is_doing = IS_THINKING

        # send zero input and update world what the robot is doing
        self.connection.send({"RBState": self.is_doing, "x": 0, "y": 0})

        print("Can I think of something...")

        # wait 2 seconds
        time.sleep(2)

        print("yes I got it! executing plan")
        self.is_doing = IS_EXECUTING

    def calculate_input(self):
        # todo: use controller/world states and other thingies to calculate the best input
        print("calculating best input")

    def send_input(self):
        # print("send input to robot")
        self.connection.send({
            "RBState": self.is_doing,
            "x": np.sin(time.localtime().tm_sec),
            "y": np.cos(time.localtime().tm_sec)
        })
