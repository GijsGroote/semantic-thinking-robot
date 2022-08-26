import warnings
import numpy as np
from scipy.spatial.transform import Rotation as R

class State:
    """
    State describing the position in the environment.
    """
    def __init__(self, pos=np.array([0, 0, 0]), vel=np.array([0, 0, 0]), acc=np.array([0, 0, 0]),
                 ang_p=np.array([0, 0, 0]), ang_v=np.array([0, 0, 0]), ang_a=np.array([0, 0, 0])):
        # position, velocity and acceleration in x and y direction
        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.ang_p = ang_p
        self.ang_v = ang_v
        self.ang_a = ang_a

    def euclidean_position(self, state):
        """
        Calculate the euclidean distance between the position of 2 states
        """
        return np.linalg.norm(self.pos - state.pos)


    def to_string(self, decimals=2):
        return "pos:("+str(np.round(self.pos, decimals)) + "), vel:("\
            + str(np.round(self.vel, decimals)) + ")\n" + "acc:(" \
            + str(np.round(self.acc, decimals)) + "), ang_p:("\
            + str(np.round(self.ang_p, decimals)) + ")\nang_v:("\
            + str(np.round(self.ang_v, decimals)) + "), ang_a:("\
            + str(np.round(self.ang_a, decimals)) + ")\n"

    def get_2d_pose(self):
        return np.array([self.pos[0], self.pos[1], self.ang_p[2]])

    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, value):
        try:
            if value.shape[0] == 3:
                self._pos = value
            elif value.shape[0] == 2:
                warnings.warn(f"shape of position is: {value.shape}, and should be (3,)")
                self._pos = np.array([value[0], value[1], 0])
            else:
                raise Exception("position has incorrect dimensions")
        except (AttributeError, TypeError, IndexError) as exc:
            raise exc

    @property
    def vel(self):
        return self._vel

    @vel.setter
    def vel(self, value):
        try:
            if value.shape[0] == 3:
                self._vel = value
            elif value.shape[0] == 2:
                warnings.warn(f"shape of velocity is: {value.shape}, and should be (3,).")
                self._vel = np.array([value[0], value[1], 0])
            else:
                raise Exception("velocity has incorrect dimensions")
        except(AttributeError, TypeError, IndexError) as exc:
            raise exc
            
    @property
    def acc(self):
        return self._acc

    @acc.setter
    def acc(self, value):
        try:
            if value.shape[0] == 3:
                self._acc = value
            elif value.shape[0] == 2:
                warnings.warn(f"shape of acceleration is: {value.shape}, and should be (3,).")
                self._acc = np.array([value[0], value[1], 0])
            else:
                raise Exception("acceleration has incorrect dimensions")
        except(AttributeError, TypeError, IndexError) as exc:
            raise exc

    @property
    def ang_p(self):
        return self._ang_p

    @ang_p.setter
    def ang_p(self, value):
        try:
            if isinstance(value, np.float64):
                self._ang_p = np.array([0, 0, value])
            elif value.shape[0] == 3:
                self._ang_p = value
            elif value.shape[0] == 4:
                self._ang_p = R.from_quat(value)
            else:
                raise Exception("angular position has incorrect dimensions")
        except(AttributeError, TypeError, IndexError) as exc:
            raise exc

    @property
    def ang_v(self):
        return self._ang_v

    @ang_v.setter
    def ang_v(self, value):
        try:
            if isinstance(value, np.float64):
                self._ang_v = np.array([0, 0, value])
            elif value.shape[0] == 3:
                self._ang_v = value
            else:
                raise Exception("angular velocity has incorrect dimensions")
        except(AttributeError, TypeError, IndexError):
            raise exc

    @property
    def ang_a(self):
        return self._ang_a

    @ang_a.setter
    def ang_a(self, value):
        try:
            if isinstance(value, np.float64):
                self._ang_a = np.array([0, 0, value])
            elif value.shape[0] == 3:
                self._ang_a = value
            else:
                raise Exception("angular acceleration has incorrect dimensions")
        except(AttributeError, TypeError, IndexError) as exc:
            raise exc
