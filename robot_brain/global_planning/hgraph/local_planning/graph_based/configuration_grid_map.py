from abc import ABC, abstractmethod
import numpy as np
import math
import warnings

from robot_brain.obstacle import Obstacle 

def check_floats_divisible(x: float, y: float, scaling_factor: float = 1e4):

    scaled_x = int(x * scaling_factor)
    scaled_y = int(y * scaling_factor)

    return (scaled_x % scaled_y) == 0

class ConfigurationGridMap(ABC):
    """ Configuration grid map represents the environment in obstacle space
    free space, movable obstacle space and unknown obstacle space and includes
    the dimensions or the robot.

    The center represents the origin, an configuration grid map can only
    take rectangular sizes where the origin is in the center.
    """

    def __init__(self,
            cell_size: float,
            grid_x_length: float,
            grid_y_length: float,
            obstacles: {},
            robot_cart_2d: np.ndarray,
            n_orientations: int
            ):

        # assert the grid can be descritized in square cells
        if (not check_floats_divisible(grid_x_length, cell_size)  or
               not check_floats_divisible(grid_y_length, cell_size)):
            raise ValueError("grid_x_length and grid_y_length must be devisable by the cell_width")

        self._cell_size = cell_size
        self._grid_x_length = grid_x_length
        self._grid_y_length = grid_y_length
        self._obstacles = obstacles 
        assert robot_cart_2d.shape == (2,), \
                f"robot position should be of shape (2,), it's: {robot_cart_2d.shape}"
        self._robot_cart_2d = robot_cart_2d

        self._n_orientations = n_orientations

    def setup(self):
        """
        Setup the gridmap for a number of obstacles.

        indices in the grid_map indicat the following:
            0 -> free space
            1 -> obstacle space
            2 -> movable obstacle space
            3 -> unknown obstacle space
        """
        for r_orien_idx in range(self.n_orientations):
            for obst in self.obstacles.values():
                
                match obst.type:
                    case "unmovable":
                        self.setup_obstacle(obst, 1, 2*math.pi*r_orien_idx/self.n_orientations, r_orien_idx)

                    case "movable":
                        self.setup_obstacle(obst, 2, 2*math.pi*r_orien_idx/self.n_orientations, r_orien_idx)

                    case "unknown":
                        self.setup_obstacle(obst, 3, 2*math.pi*r_orien_idx/self.n_orientations, r_orien_idx)

                    case _:
                        raise TypeError(f"unknown type: {obst.type}")
  
    def setup_obstacle(self, obst: Obstacle, val: int, r_orien: float, r_orien_idx: int):
        """ Set the obstect overlapping with grid cells to a integer value. """ 

        match obst.properties.type():
            # TODO: create a shadow function, which removes the need for warnings when unusual orienatations occur
            case "cylinder":
                # "obstects" x-axis is parallel to the global z-axis (normal situation)
                if not (math.isclose(math.sin(obst.state.ang_p[0]), 0, abs_tol=0.01) and 
                        math.isclose(math.sin(obst.state.ang_p[1]), 0, abs_tol=0.01)):
                    warnings.warn(f"obstacle {obst.name} is not in correct orientation (up/down is not up)")

                self.setup_circle_obstacle(obst, val, r_orien, r_orien_idx)

            case "sphere":
                self.setup_circle_obstacle(obst, val, r_orien, r_orien_idx)

            case "box":
                # "obstects" x-axis is parallel to the global z-axis (normal situation)
                if not ((math.isclose(obst.state.ang_p[0], 0, abs_tol=0.01) or 
                        math.isclose(obst.state.ang_p[0], 2*math.pi, abs_tol=0.01)) and
                        math.isclose(obst.state.ang_p[1], 0, abs_tol=0.01) or
                        math.isclose(obst.state.ang_p[1], 2*math.pi, abs_tol=0.01)):
                    warnings.warn(f"obstacle {obst.name} is not in correct orientation (up is not up)")

                self.setup_rectangular_obstacle(obst, val, r_orien, r_orien_idx)

            case "urdf":
                warnings.warn(f"the urdf type is not yet implemented")

            case _:
                
                raise TypeError(f"Could not recognise obstacle type: {obst.properties.type()}")

    @abstractmethod
    def setup_rectangular_obstacle(self, obst: Obstacle, val: int, r_orien: float, r_orien_idx: int):
        pass

    @abstractmethod
    def setup_circle_obstacle(self, obst: Obstacle, val: int, r_orien: float, r_orien_idx: int):
        pass

    @abstractmethod
    def occupancy(self, y_position, x_position, *args):
        pass

    def c_idx_to_cart_2d(self, x_idx: int, y_idx: int) -> (float, float):
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

    def cart_2d_to_c_idx_or_grid_edge(self, x_position: float, y_position: float) -> (int, int):
        """ returns the index of the cell a position is in,
        if the position is outside the boundary of the grid map the edges
        will be returned.
        """
        try:
            x_idx = self.cart_2d_to_c_idx(x_position, 0)[0]
        except IndexError as exc:
            if x_position > self.grid_x_length/2:
                x_idx = int(self.grid_x_length/self.cell_size-1)
            elif x_position < -self.grid_x_length/2:
                x_idx = 0
            else:
                raise IndexError(f"x_position: {x_position} "\
                        "could not be converted to cell index.") from exc

        try:
            y_idx = self.cart_2d_to_c_idx(0, y_position)[1]
        except IndexError as exc:
            if y_position > self.grid_y_length/2:
                y_idx = int(self.grid_y_length/self.cell_size-1)
            elif y_position < -self.grid_y_length/2:
                y_idx = 0
            else:
                raise IndexError(f"y_position: {y_position} "\
                        "could not be converted to cell index.") from exc

        return (x_idx, y_idx)

    def cart_2d_to_c_idx(self, x_position: float, y_position: float) -> (int, int):
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
    def obstacles(self):
        return self._obstacles

    @obstacles.setter
    def obstacles(self, obstacles):
        assert isinstance(obstacles, dict), f"obstacles should be a dictionary and is {type(obstacles)}"
        self._obstacles = obstacles 

    @property
    def robot_cart_2d(self):
        return self._robot_cart_2d

    @property
    def n_orientations(self):
        return self._n_orientations
