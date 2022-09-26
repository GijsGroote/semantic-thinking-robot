from abc import ABC, abstractmethod
import numpy as np

class OccupancyMap(ABC):
    """ Occupancy map represents the environment in obstacle space
    free space, movable obstacle space and unknown obstacle space.
    """

    def __init__(self,
            cell_size: float,
            grid_length: float,
            grid_width: float):

        # assert the grid can be descritized in square cells
        if (grid_length % cell_size != 0 or
                grid_width % cell_size != 0):
            raise ValueError("grid_length and grid_width must be devisable by the cell_width")

        self._cell_size = cell_size
        self._grid_length = grid_length
        self._grid_width = grid_width

    @abstractmethod
    def setup(self, obstacles):
        pass

    @abstractmethod
    def occupancy(self, x_position, y_position, *args):
        pass

    @property
    def cell_size(self):
        return self._cell_size

    @property
    def grid_length(self):
        return self._grid_length

    @property
    def grid_width(self):
        return self._grid_width
