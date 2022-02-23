import numpy as np

class State:

    def __init__(self, pos_x=0, pos_y=0, vel_x=0, vel_y=0, acc_x=0, acc_y=0, ang_p=0, ang_v=0, ang_a=0):
        # position, velocity and acceleration in x and y direction
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.ang_p = ang_p
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.ang_v = ang_v
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.ang_a = ang_a

    def update_pos(self, arr):
        """
        Updates the state position.
        :param arr: 3 X 1 array
        :return:
        """
        self._pos_x = arr[0]
        self._pos_y = arr[1]
        self._ang_p = arr[2]

    def update_vel(self, arr):
        """
        Updates the state velocity.
        :param arr: 3 X 1 array
        :return:
        """
        self.vel_x = arr[0]
        self.vel_y = arr[1]
        self.ang_v = arr[2]


    def update_acc(self, arr):
        """
        Updates the state acceleration.
        :param arr: 3 X 1 array
        :return:
        """
        self.acc_x = arr[0]
        self.acc_y = arr[1]
        self.ang_a = arr[2]

    def euclidean(self, state):
        """
        Calculate the euclidean distance between 2 states

        :param state:
        :return:

        """
        # create array of state objects
        arr1 = np.array([self.pos_x, self.pos_y])
        arr2 = np.array([state.pos_x, state.pos_y])

        return np.linalg.norm(arr1-arr2)

    def obj2arr(self):
        return

    @property
    def pos_x(self):
        return self._pos_x

    @pos_x.setter
    def pos_x(self, value):
        self._pos_x = value

    @property
    def pos_y(self):
        return self._pos_y

    @pos_y.setter
    def pos_y(self, value):
        self._pos_y = value

    @property
    def vel_x(self):
        return self._vel_x

    @vel_x.setter
    def vel_x(self, value):
        self._vel_x = value

    @property
    def vel_y(self):
        return self._vel_y

    @vel_y.setter
    def vel_y(self, value):
        self._vel_y = value

    @property
    def acc_x(self):
        return self._acc_x

    @acc_x.setter
    def acc_x(self, value):
        self._acc_x = value

    @property
    def acc_y(self):
        return self._acc_y

    @acc_y.setter
    def acc_y(self, value):
        self._acc_y = value

    @property
    def ang_p(self):
        return self._ang_p

    @ang_p.setter
    def ang_p(self, value):
        self._ang_p = value

    @property
    def ang_v(self):
        return self._ang_v

    @ang_v.setter
    def ang_v(self, value):
        self._ang_v = value

    @property
    def ang_a(self):
        return self._ang_a

    @ang_a.setter
    def ang_a(self, value):
        self._ang_a = value
