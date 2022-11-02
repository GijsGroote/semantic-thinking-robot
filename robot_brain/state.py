import warnings
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

class State:
    """
    State describing the linear and angular position and velocity in the environment.
    """
    def __init__(self, pos=np.array([0, 0, 0]), vel=np.array([0, 0, 0]),
                 ang_p=np.array([0, 0, 0]), ang_v=np.array([0, 0, 0])):

        self.pos = pos
        self.vel = vel
        self.ang_p = ang_p
        self.ang_v = ang_v

    def euclidean(self, state) -> float:
        """
        Calculate the euclidean distance between two states.
        """
        return (np.linalg.norm(self.pos - state.pos) + np.linalg.norm(self.ang_p - state.ang_p)
            + np.linalg.norm(self.vel- state.vel) + np.linalg.norm(self.ang_v- state.ang_v)).item()

    def pose_euclidean(self, state) -> float:
        """
        Calculate the euclidean distance between the position of two states.
        """
        return np.linalg.norm(self.get_2d_pose() - state.get_2d_pose()).item()

    def to_string(self, decimals=2):
        return "pos:("+np.array2string(self.pos, precision=decimals)+"), vel:("\
                +np.array2string(self.vel, precision=decimals)+"), ang_p:("\
                +np.array2string(self.ang_p, precision=decimals)+"), ang_v:("\
                +np.array2string(self.ang_v, precision=decimals)+")."

    def get_2d_pose(self):
        return np.array([self.pos[0], self.pos[1], self.ang_p[2]])

    def get_xy_position(self):
        return np.array([self.pos[0], self.pos[1]])

    def get_xy_dxdy(self):
        return np.array([self.pos[0], self.pos[1], self.vel[0], self.vel[1]])

    def get_xyt_dxdydt(self):
        return np.array([self.pos[0], self.pos[1], self.ang_p[2], self.vel[0], self.vel[1], self.ang_v[2]])

    def lies_on_a_side(self) -> bool:
        """ indicates if one (x,y,z) points perpendicular to the ground plane. """

        assert self.ang_p.shape == (3,), "angular position should be of shape (3,)."

        return ((math.isclose(math.sin(self.ang_p[0]), 0, abs_tol=0.01) and
            math.isclose(math.sin(self.ang_p[1]), 0, abs_tol=0.01)) or
                (math.isclose(math.cos(self.ang_p[0]), 0, abs_tol=0.01) and
                    math.isclose(math.sin(self.ang_p[2]), 0, abs_tol=0.01)) or
                (math.isclose(math.cos(self.ang_p[1]), 0, abs_tol=0.01) and
                    math.isclose(math.sin(self.ang_p[0]), 0, abs_tol=0.01)))
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
                rot = R.from_quat(value)
                self._ang_p = rot.as_euler('xyz', degrees=False)
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
        except(AttributeError, TypeError, IndexError) as exc:
            raise exc
