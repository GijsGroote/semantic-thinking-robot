from abc import ABC, abstractmethod
import numpy as np

class OccupancyMap(ABC):
    """ Occupancy map represents the environment in obstacle space
    free space, movable obstacle space and unknown obstacle space.
    """

    def __init__(self,
            cell_width: float,
            grid_height: float,
            grid_width: float):

        # assert the grid can be descritized in square cells
        if (grid_height % cell_width != 0 or
                grid_width % cell_width != 0):
            raise ValueError("grid_height and grid_width must be devisable by the cell_width")

        self._cell_width = cell_width
        self._grid_height = grid_height
        self._grid_width = grid_width


    @abstractmethod
    def setup(self, obstacles):
        pass

    @abstractmethod
    def occupancy(self, x_position, y_position, **args):
        pass

    @property
    def cell_width(self):
        return self._cell_width

    @property
    def grid_height(self):
        return self._grid_height

    @property
    def grid_width(self):
        return self._grid_width
