import numpy as np
from robot_brain.planning.grid_map.occupancy_map import OccupancyMap


class CircleRobotOccupancyMap(OccupancyMap):
    """ 2 dimensional occupancy map representing the environment
    in obstacle space, free space, movable obstacle space and
    unknown obstacle space for a circular robot.
    """

    def __init__(self,
            cell_width: float,
            grid_height: float,
            grid_width: float,
            robot_radius: float):

        OccupancyMap.__init__(self, cell_width, grid_height, grid_width)
        self._robot_radius = robot_radius
        self._grid_map = np.zeros((
            int(self.grid_width/self.cell_width),
            int(self.grid_height/self.cell_width)))

    def setup(self, obstacles):
        raise NotImplementedError()

    def occupancy(self, x_position, y_position, **args):
        return self._grid_map[x_position, y_position]

    @property
    def grid_map(self):
        return self._grid_map

    @property
    def robot_radius(self):
        return self._robot_radius

