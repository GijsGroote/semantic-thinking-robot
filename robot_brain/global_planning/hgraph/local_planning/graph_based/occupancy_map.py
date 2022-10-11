from abc import ABC, abstractmethod
import numpy as np
import math
import warnings

from robot_brain.object import Object

def check_floats_divisible(x: float, y: float, scaling_factor: float = 1e4):

    scaled_x = int(x * scaling_factor)
    scaled_y = int(y * scaling_factor)

    return (scaled_x % scaled_y) == 0

class OccupancyMap(ABC):
    """ Occupancy map represents the environment in obstacle space
    free space, movable obstacle space and unknown obstacle space.

    The center represents the origin, an occupancy map can only
    take rectangular sizes where the origin is in the center.
    """

    def __init__(self,
            cell_size: float,
            grid_x_length: float,
            grid_y_length: float,
            objects: dict,
            robot_position: np.ndarray,
            n_orientations: int
            ):

        # assert the grid can be descritized in square cells
        if (not check_floats_divisible(grid_x_length, cell_size)  or
               not check_floats_divisible(grid_y_length, cell_size)):
            raise ValueError("grid_x_length and grid_y_length must be devisable by the cell_width")

        self._cell_size = cell_size
        self._grid_x_length = grid_x_length
        self._grid_y_length = grid_y_length
        self._objects = objects

        assert robot_position.shape == (2,) or robot_position.shape == (3,), \
                f"robot position should be of shape (2,), it's: {robot_position.shape}"
        self._robot_position = robot_position

        self._n_orientations = n_orientations

    def setup(self):
        """
        Setup the gridmap for a number of obstacles.

        indices in the grid_map indicat the following:
            0 -> free space
            1 -> obstacle space
            2 -> movable object space
            3 -> unknown object space
        """
        for i_r_orien in range(self.n_orientations):
            
            for obj in self.objects.values():
                match obj.type:
                    case "unmovable":
                        self.setup_object(obj, 1, 2*math.pi*i_r_orien/self.n_orientations, i_r_orien)

                    case "movable":
                        self.setup_object(obj, 2, 2*math.pi*i_r_orien/self.n_orientations, i_r_orien)

                    case "unknown":
                        self.setup_object(obj, 3, 2*math.pi*i_r_orien/self.n_orientations, i_r_orien)

                    case _:
                        raise TypeError(f"unknown type: {obj.type}")
  
    def setup_object(self, obj: Object, val: int, r_orien: float, i_r_orien: int):
        """ Set the object overlapping with grid cells to a integer value. """ 

        match obj.obstacle.type():
            case "cylinder":
                # "objects" x-axis is parallel to the global z-axis (normal situation)
                if not (math.isclose(math.sin(obj.state.ang_p[0]), 0, abs_tol=0.01) and 
                        math.isclose(math.sin(obj.state.ang_p[1]), 0, abs_tol=0.01)):
                    warnings.warn(f"obstacle {obj.name} is not in correct orientation (up/down is not up)")

                self.setup_circular_object(obj, val, r_orien, i_r_orien)

            case "sphere":
                self.setup_circular_object(obj, val, r_orien, i_r_orien)

            case "box":
                # "objects" x-axis is parallel to the global z-axis (normal situation)
                if not ((math.isclose(obj.state.ang_p[0], 0, abs_tol=0.01) or 
                        math.isclose(obj.state.ang_p[0], 2*math.pi, abs_tol=0.01)) and
                        math.isclose(obj.state.ang_p[1], 0, abs_tol=0.01) or
                        math.isclose(obj.state.ang_p[1], 2*math.pi, abs_tol=0.01)):
                    warnings.warn(f"obstacle {obj.name} is not in correct orientation (up is not up)")

                self.setup_rectangular_object(obj, val, r_orien, i_r_orien)

            case "urdf":
                warnings.warn(f"the urdf type is not yet implemented")

            case _:
                
                raise TypeError(f"Could not recognise obstacle type: {obj.obstacle.type()}")

    @abstractmethod
    def setup_rectangular_object(self, obj: object, val: int, r_orien: float, i_r_orien: int):
        pass

    @abstractmethod
    def setup_circular_object(self, obj: Object, val: int, r_orien: float, i_r_orien: int):
        pass

    @abstractmethod
    def occupancy(self, y_position, x_position, *args):
        pass

    def cell_idx_to_position(self, x_idx: int, y_idx: int) -> (float, float):
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

    def position_to_cell_idx_or_grid_edge(self, x_position: float, y_position: float) -> (int, int):
        """ returns the index of the cell a position is in,
        if the position is outside the boundary of the grid map the edges
        will be returned.
        """
        try:
            x_idx = self.position_to_cell_idx(x_position, 0)[0]
        except IndexError as exc:
            if x_position > self.grid_x_length/2:
                x_idx = int(self.grid_x_length/self.cell_size-1)
            elif x_position < -self.grid_x_length/2:
                x_idx = 0
            else:
                raise IndexError(f"x_position: {x_position} "\
                        "could not be converted to cell index.") from exc

        try:
            y_idx = self.position_to_cell_idx(0, y_position)[1]
        except IndexError as exc:
            if y_position > self.grid_y_length/2:
                y_idx = int(self.grid_y_length/self.cell_size-1)
            elif y_position < -self.grid_y_length/2:
                y_idx = 0
            else:
                raise IndexError(f"y_position: {y_position} "\
                        "could not be converted to cell index.") from exc

        return (x_idx, y_idx)

    def position_to_cell_idx(self, x_position: float, y_position: float) -> (int, int):
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
    def objects(self):
        return self._objects

    @property
    def robot_position(self):
        return self._robot_position

    @property
    def n_orientations(self):
        return self._n_orientations
