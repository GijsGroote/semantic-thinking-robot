import numpy as np
import warnings


class State:

<<<<<<< HEAD
    def __init__(self, pos_x=0, pos_y=0, ang_p=0, vel_x=0, vel_y=0, ang_v=0, acc_x=0, acc_y=0, ang_a=0):
        # position, velocity and acceleration in x, y and angular direction
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.ang_p = ang_p
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.ang_v = ang_v
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.ang_a = ang_a

    def update_p_and_v(self, pos_x, pos_y, ang_p, vel_x, vel_y, ang_v):
        # position, velocity and acceleration in x, y and angular direction
        self.pos_x = pos_x
        self.pos_y = pos_y
=======
    def __init__(self, pos=np.array([0, 0, 0]), vel=np.array([0, 0, 0]), acc=np.array([0, 0, 0]),
                 ang_p=np.array([0, 0, 0]), ang_v=np.array([0, 0, 0]), ang_a=np.array([0, 0, 0])):
        # position, velocity and acceleration in x and y direction
        self.pos = pos
        self.vel = vel
        self.acc = acc
>>>>>>> main
        self.ang_p = ang_p
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.ang_v = ang_v


    def euclidean_position(self, state):
        """
<<<<<<< HEAD
        # create array of state objects
        # todo: this really is not how to handle variable array's
        arr1 = np.array([self.pos_x, self.pos_y])
        arr2 = np.array([state.pos_x, state.pos_y])

        return np.linalg.norm(arr1-arr2)

    def print(self):
        print(self.toString())

    def toString(self):
        return "position: ({}, {}, {})\nlocation: ({}, {}, {})\nangular: ({}, {}, {})"\
            .format(self.pos_x, self.pos_y, self.ang_p, self.vel_x, self.vel_y, self.ang_v, self.acc_x, self.acc_y, self.ang_a)

    def obj2arr(self):
        return

    def pos2arr(self):
        return np.array([self.pos_x, self.pos_y, self.ang_p])

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
=======
        Calculate the euclidean distance between the position of 2 states
        """
        return np.linalg.norm(self.pos - state.pos)
>>>>>>> main

    def toString(self, d=2):
        return "pos:("+str(np.round(self.pos, d)) + "), vel:(" + str(np.round(self.vel, d)) + ")\n" \
                + "acc:(" + str(np.round(self.acc, d)) + "), ang_p:("+str(np.round(self.ang_p, d)) \
               + ")\nang_v:(" + str(np.round(self.ang_v, d)) + "), acc:(" + str(np.round(self.ang_a, d)) + ")\n"

    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, value):
        try:
            if value.shape[0] == 3:
                self._pos = value
            elif value.shape[0] == 2:
                warnings.warn("shape of position is: {}, and should be (3,). Converting shape".format(value.shape))
                self._pos = np.array([value[0], value[1], 0])
            else:
                raise Exception("position has incorrect dimensions")
        except(AttributeError, TypeError, IndexError):
            raise AssertionError("input should be an numpy array")

    @property
    def vel(self):
        return self._vel

    @vel.setter
    def vel(self, value):
        try:
            if value.shape[0] == 3:
                self._vel = value
            elif value.shape[0] == 2:
                warnings.warn("shape of velocity is: {}, and should be (3,). Converting shape".format(value.shape))
                self._vel = np.array([value[0], value[1], 0])
            else:
                raise Exception("velocity has incorrect dimensions")
        except(AttributeError, TypeError, IndexError):
            raise AssertionError("input should be an numpy array")

    @property
    def acc(self):
        return self._acc

    @acc.setter
    def acc(self, value):
        try:
            if value.shape[0] == 3:
                self._acc = value
            elif value.shape[0] == 2:
                warnings.warn("shape of acceleration is: {}, and should be (3,). Converting shape".format(value.shape))
                self._acc = np.array([value[0], value[1], 0])
            else:
                raise Exception("acceleration has incorrect dimensions")
        except(AttributeError, TypeError, IndexError):
            raise AssertionError("input should be an numpy array")


    @property
    def ang_p(self):
        return self._ang_p

    @ang_p.setter
    def ang_p(self, value):
        try:
            if value.shape[0] == 3:
                self._ang_p = value
            else:
                raise Exception("angular position has incorrect dimensions")
        except(AttributeError, TypeError, IndexError):
            raise AssertionError("input should be an numpy array")

    @property
    def ang_v(self):
        return self._ang_v

    @ang_v.setter
    def ang_v(self, value):
        try:
            if value.shape[0] == 3:
                self._ang_v = value
            else:
                raise Exception("angular velocity has incorrect dimensions")
        except(AttributeError, TypeError, IndexError):
            raise AssertionError("input should be an numpy array")

    @property
    def ang_a(self):
        return self._ang_a

    @ang_a.setter
    def ang_a(self, value):
        try:
            if value.shape[0] == 3:
                self._ang_a = value
            else:
                raise Exception("angular acceleration has incorrect dimensions")
        except(AttributeError, TypeError, IndexError):
            raise AssertionError("input should be an numpy array")

