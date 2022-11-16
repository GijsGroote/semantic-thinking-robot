from abc import ABC, abstractmethod

class RRT(ABC):
    """
    Doubly Rapid Random Tree star (RRT*) motion planner.
    """
    def __init__(self, 
        grid_x_length: float,
        grid_y_length: float,
        obstacles: dict):

       
        assert isinstance(grid_x_length, float), f"grid_x_length must be type float and is type {type(grid_x_length)}"
        assert isinstance(grid_y_length, float), f"grid_y_length must be type float and is type {type(grid_y_length)}"
        assert grid_x_length > 0 and grid_y_length > 0, f"both grid_x_length and grid_y_length must be positive they are {grid_x_length} and {grid_y_length}"

        self._grid_x_length = grid_x_length
        self._grid_y_length = grid_y_length
        self.obstacles = obstacles


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


