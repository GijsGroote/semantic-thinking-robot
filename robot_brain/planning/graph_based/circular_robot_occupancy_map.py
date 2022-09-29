import numpy as np
from robot_brain.planning.graph_based.occupancy_map import OccupancyMap

class CircleRobotOccupancyMap(OccupancyMap):
    """ 2 dimensional occupancy map representing the environment
    in obstacle space, free space, movable obstacle space and
    unknown obstacle space for a circular robot.
    """

    def __init__(self,
            cell_size: float,
            grid_x_length: float,
            grid_y_length: float,
            robot_radius: float):

        OccupancyMap.__init__(self, cell_size, grid_x_length, grid_y_length)
        self._robot_radius = robot_radius
        self._grid_map = np.zeros((
            int(self.grid_x_length/self.cell_size),
            int(self.grid_y_length/self.cell_size)))

    def setup(self, obstacles):
        raise NotImplementedError()

    def cell_idx_to_position(self, x_idx: int, y_idx: int) -> (float, float):
        raise NotImplementedError()

    def position_to_cell_idx(self, x_position: float, y_position: float) -> (int, int):
        raise NotImplementedError()

    def occupancy(self, x_position, y_position, *args):
        return self._grid_map[x_position, y_position]

    @property
    def grid_map(self):
        return self._grid_map

    @property
    def robot_radius(self):
        return self._robot_radius
