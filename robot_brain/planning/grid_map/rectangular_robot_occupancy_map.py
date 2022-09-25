import numpy as np
from robot_brain.planning.grid_map.occupancy_map import OccupancyMap

class RectangularRobotOccupancyMap(OccupancyMap):
    """ Occupancy map represents the environment in obstacle space
    free space, movable obstacle space and unknown obstacle space.
    """

    def __init__(self,
        cell_width: float,
        grid_height: float,
        grid_width: float,
        robot_heigth: float,
        robot_width: float,
        n_orientations: int):

        OccupancyMap.__init__(self, cell_width, grid_height, grid_width)
        self._robot_height = robot_heigth
        self._robot_width = robot_width
        self._grid_map = np.zeros((
            int(self.grid_width/self.cell_width),
            int(self.grid_height/self.cell_width,
            n_orientations)))

    def setup(self, obstacles):
        raise NotImplementedError()

    def occupancy(self, x_position, y_position, orientation):
        return self.grid_map[x_position, y_position, orientation]

    @property
    def grid_map(self):
        return self._grid_map
