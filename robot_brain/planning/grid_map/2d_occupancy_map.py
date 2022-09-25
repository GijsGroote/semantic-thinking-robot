import numpy as np


class OccupancyMap2D:
    """ 2 dimensional occupancy map representing the environment
    in obstacle space, free space, movable obstacle space and
    unknown obstacle space for a fixed orientation of the robot.
    """
    def __init__(self, n_grid_cells_width, n_grid_cells_height, robot_orientation=None):

        self._robot_orientation = robot_orientation
        self._2d_grid_map = np.zeros((n_grid_cells_width, n_grid_cells_height))

