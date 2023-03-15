from abc import ABC, abstractmethod
import math
import warnings
from typing import Tuple
import numpy as np

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.obstacle import Obstacle, MOVABLE, UNKNOWN, UNMOVABLE
from helper_functions.geometrics import check_floats_divisible

# TODO: make the path estimator focussed on an obstacle (and not the robot)
# see the variables r_idx that stand for robot idx
class PathEstimator(ABC):
    """ With as internal structure a configuration grid map, the path estimator
    represents the environment in obstacle space free space, movable obstacle
    space and unknown obstacle space and includes
    the dimensions or the robot.

    The center represents the origin, an configuration grid map can only
    take rectangular sizes where the origin is in the center.
    """

    def __init__(self,
            cell_size: float,
            grid_x_length: float,
            grid_y_length: float,
            obstacles: dict,
            obst_cart_2d: np.ndarray,
            obst_name: str,
            n_orientations: int,
            single_orientation: bool,
            orientation: float):

        # assert the grid can be descritized in square cells
        if (not check_floats_divisible(grid_x_length, cell_size)  or
               not check_floats_divisible(grid_y_length, cell_size)):
            raise ValueError("grid_x_length and grid_y_length must be devisable by the cell_width")

        self._cell_size = cell_size
        self._grid_x_length = grid_x_length
        self._grid_y_length = grid_y_length
        self._obstacles = obstacles
        assert obst_cart_2d.shape == (2,), \
                f"obstacle position should be of shape (2,), it's: {obst_cart_2d.shape}"
        self._obst_cart_2d = obst_cart_2d
        self.obst_name = obst_name

        if single_orientation:
            self._n_orientations = 1
        else:
            self._n_orientations = n_orientations

        self.single_orientation = single_orientation
        self.orientation = orientation
        self.shortest_path = None

    @abstractmethod
    def search_path(self, cart_2d_start: np.ndarray, cart_2d_target: np.ndarray) -> list:
        """ return shortest path and boolean indicating if a shortest path can be found. """

    def setup(self):
        """
        Setup the gridmap for a number of obstacles.

        indices in the grid_map indicat the following:
            0 -> free space
            1 -> movable space
            2 -> unknown space
            3 -> obstacle space
        """
        for r_orien_idx in range(self.n_orientations):

            # if a specific orientation is requested, only create that one
            if self.single_orientation:
                r_orien = self.orientation
            else:
                r_orien = 2*math.pi*r_orien_idx/self.n_orientations

            # setup movable obstacles
            for obst in self.obstacles.values():
                # checking for an obstacle, exclude itself from the list
                if obst.name == self.obst_name:
                    continue
                if obst.type==MOVABLE:
                    self._setup_obstacle(obst, MOVABLE, r_orien, r_orien_idx)

            # setup unknown obstacles
            for obst in self.obstacles.values():
                if obst.name == self.obst_name:
                    continue
                if obst.type==UNKNOWN:
                    self._setup_obstacle(obst, UNKNOWN, r_orien, r_orien_idx)

            # setup unmovable obstacles
            for obst in self.obstacles.values():
                # checking for an obstacle, exclude itself from the list
                if obst.name == self.obst_name:
                    continue
                if obst.type==UNMOVABLE:
                    self._setup_obstacle(obst, UNMOVABLE, r_orien, r_orien_idx)

    def _setup_obstacle(self, obst: Obstacle, val: int, r_orien: float, r_orien_idx: int):
        """ Set the obstect overlapping with grid cells to a integer value. """

        if isinstance(obst.properties, CylinderObstacle):
            # "obstects" x-axis is parallel to the global z-axis (normal situation)
            if not (math.isclose(math.sin(obst.state.ang_p[0]), 0, abs_tol=0.01) and
                    math.isclose(math.sin(obst.state.ang_p[1]), 0, abs_tol=0.01)):
                warnings.warn(f"obstacle {obst.name} is not in correct orientation (up is not up)")

            self._setup_circle_obstacle(obst, val, r_orien, r_orien_idx)

        elif isinstance(obst.properties, BoxObstacle):
            # "obstects" x-axis is parallel to the global z-axis (normal situation)
            if not ((math.isclose(obst.state.ang_p[0], 0, abs_tol=0.01) or
                    math.isclose(obst.state.ang_p[0], 2*math.pi, abs_tol=0.01)) and
                    math.isclose(obst.state.ang_p[1], 0, abs_tol=0.01) or
                    math.isclose(obst.state.ang_p[1], 2*math.pi, abs_tol=0.01)):
                warnings.warn(f"obstacle {obst.name} is not in correct orientation (up is not up)")

            self._setup_rectangular_obstacle(obst, val, r_orien, r_orien_idx)

        else:
            raise TypeError(f"Could not recognise obstacle type: {obst.properties.type()}")

    @abstractmethod
    def _setup_rectangular_obstacle(self, obst: Obstacle, val: int, r_orien: float, r_orien_idx: int):
        pass

    @abstractmethod
    def _setup_circle_obstacle(self, obst: Obstacle, val: int, r_orien: float, r_orien_idx: int):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def occupancy(self, pose_2d: np.ndarray) -> int:
        pass

    def _c_idx_to_cart_2d(self, x_idx: int, y_idx: int) -> Tuple[float, float]:
        """ returns the center position that the cell represents. """

        if (x_idx >= self.grid_x_length/self.cell_size or
                x_idx < 0):
            raise IndexError(f"x_idx: {x_idx} is larger than the grid"\
                    f" [0, {int(self.grid_x_length/self.cell_size-1)}]")
        if (abs(y_idx) >= self.grid_y_length/self.cell_size or
                y_idx < 0):
            raise IndexError(f"y_idx: {y_idx} is larger than the grid"\
                    f" [0, {int(self.grid_y_length/self.cell_size-1)}]")

        return (self.cell_size*(0.5+x_idx) - self.grid_x_length/2,
                self.cell_size*(0.5+y_idx) - self.grid_y_length/2)

    def _cart_2d_to_c_idx_or_grid_edge(self, x_position: float, y_position: float) -> Tuple[int, int]:
        """ returns the index of the cell a position is in,
        if the position is outside the boundary of the grid map the edges
        will be returned.
        """
        try:
            x_idx = self._cart_2d_to_c_idx(x_position, 0)[0]
        except IndexError as exc:
            if x_position > self.grid_x_length/2:
                x_idx = int(self.grid_x_length/self.cell_size-1)
            elif x_position < -self.grid_x_length/2:
                x_idx = 0
            else:
                raise IndexError(f"x_position: {x_position} "\
                        "could not be converted to cell index.") from exc

        try:
            y_idx = self._cart_2d_to_c_idx(0, y_position)[1]
        except IndexError as exc:
            if y_position > self.grid_y_length/2:
                y_idx = int(self.grid_y_length/self.cell_size-1)
            elif y_position < -self.grid_y_length/2:
                y_idx = 0
            else:
                raise IndexError(f"y_position: {y_position} "\
                        "could not be converted to cell index.") from exc

        return (x_idx, y_idx)

    def _cart_2d_to_c_idx(self, x_position: float, y_position: float) -> Tuple[int, int]:
        """ returns the index of the cell a position is in. """
        if abs(x_position) > self.grid_x_length/2:
            raise IndexError(f"x_position: {x_position} is larger than the grid"\
                    f" [{-self.grid_x_length/2}, {self.grid_x_length/2}]")

        if abs(y_position) > self.grid_y_length/2:
            raise IndexError(f"y_position: {y_position} is larger than the grid"\
                    f" [{-self.grid_y_length/2}, {self.grid_y_length/2}]")

        x_idx = int((x_position + self.grid_x_length/2)/self.cell_size)
        y_idx = int((y_position + self.grid_y_length/2)/self.cell_size)

        # if the cell index is exactly on the 'south' or 'east' border, decrement index
        if x_idx == int(self.grid_x_length/self.cell_size):
            x_idx -= 1
        if y_idx == int(self.grid_y_length/self.cell_size):
            y_idx -= 1

        return (x_idx, y_idx)

    @property
    def cell_size(self):
        return self._cell_size

    @property
    def grid_x_length(self):
        return self._grid_x_length

    @property
    def grid_y_length(self):
        return self._grid_y_length

    @property
    def grid_map(self):
        return self._grid_map

    @grid_map.setter
    def grid_map(self, val):
        self._grid_map = val

    @property
    def obstacles(self):
        return self._obstacles

    @obstacles.setter
    def obstacles(self, obstacles):
        assert isinstance(obstacles, dict), \
        f"obstacles should be a dictionary and is {type(obstacles)}"
        self._obstacles = obstacles

    @property
    def obst_cart_2d(self):
        return self._obst_cart_2d

    @property
    def n_orientations(self):
        return self._n_orientations
