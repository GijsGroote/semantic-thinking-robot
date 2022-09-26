import numpy as np
from robot_brain.planning.graph_based.occupancy_map import OccupancyMap

class RectangularRobotOccupancyMap(OccupancyMap):
    """ Occupancy map represents the environment in obstacle space
    free space, movable obstacle space and unknown obstacle space.
    """

    def __init__(self,
        cell_size: float,
        grid_length: float,
        grid_width: float,
        robot_length: float,
        robot_width: float,
        n_orientations: int):

        OccupancyMap.__init__(self, cell_size, grid_length, grid_width)
        self._robot_length = robot_length
        self._robot_width = robot_width
        self._n_orientations = n_orientations

        self._grid_map = np.zeros((
            int(self.grid_length/self.cell_size),
            int(self.grid_width/self.cell_size),
            n_orientations))

    def setup(self, objects):
        """
        Setup the gridmap for a number of obstacles.

        indices in the grid_map indicat the following:
            0 -> free space
            1 -> obstacle space
            2 -> movable object space
            3 -> unknown object space
        """
        for (key, obj) in objects.items():

            print(f"object name: {key}, object itself {obj}")

            match obj.type:
                case "unmovable":
                    print("add the unmovable to the grid")
                case "movable":
                    print("add to the movable grid")
                case "unknown":
                    print("add to the unknown")
                    print(f"this is the dimensions {obj.obstacle.length()}, {obj.obstacle.width()}")
                case _:
                    raise TypeError(f"unknown type: obj.type")


    def occupancy(self, x_index: int, y_index: int, *args):
        if (x_index > self.grid_map.shape[0] or
            x_index < 0):
            raise ValueError(f"x_index should be in range [0, {self.grid_map.shape[0]}] and is {x_index}")

        if (y_index > self.grid_map.shape[1] or
            y_index < 0):
            raise ValueError(f"y_index should be in range [0, {self.grid_map.shape[1]}] and is {y_index}")

        if len(args) != 1: 
            raise TypeError(f"*args must be an integer")

        if not (isinstance(args[0], int) and
            type(args[0]) is int):
            raise TypeError(f"*args must be an integer")

        i_orientation = int(args[0])

        if (i_orientation > self.n_orientations or
            i_orientation < 0):
            raise ValueError(f"*args must be an integer in range [0, {self.n_orientations}] and is {i_orientation}")


        return self.grid_map[x_index, y_index, i_orientation]

    @property
    def grid_map(self):
        return self._grid_map

    @property
    def robot_length(self):
        return self._robot_length

    @property
    def robot_width(self):
        return self._robot_width

    @property
    def n_orientations(self):
        return self._n_orientations

