from abc import ABC, abstractmethod
import numpy as np

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
            ):

        # assert the grid can be descritized in square cells
        if (not check_floats_divisible(grid_x_length, cell_size)  or
               not check_floats_divisible(grid_y_length, cell_size)):
            raise ValueError("grid_x_length and grid_y_length must be devisable by the cell_width")

        self._cell_size = cell_size
        self._grid_x_length = grid_x_length
        self._grid_y_length = grid_y_length

    @abstractmethod
    def cell_idx_to_position(self, x_idx: int, y_idx: int) -> (float, float):
        pass

    @abstractmethod
    def position_to_cell_idx(self, x_position: float, y_position: float) -> (int, int):
        pass

    @abstractmethod
    def setup(self, obstacles):
        pass

    @abstractmethod
    def occupancy(self, y_position, x_position, *args):
        pass

    @property
    def cell_size(self):
        return self._cell_size

    @property
    def grid_x_length(self):
        return self._grid_x_length

    @property
    def grid_y_length(self):
        return self._grid_y_length
    
    # TODO: want to enforce things as visualise? 
