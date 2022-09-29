import numpy as np
from robot_brain.planning.graph_based.occupancy_map import OccupancyMap
import math
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as go

from robot_brain.planning.object import Object

class RectangularRobotOccupancyMap(OccupancyMap):
    """ Occupancy map represents the environment in obstacle space
    free space, movable obstacle space and unknown obstacle space.

    """

    def __init__(self,
        cell_size: float,
        grid_x_length: float,
        grid_y_length: float,
        robot_x_length: float,
        robot_y_length: float,
        n_orientations: int):

        OccupancyMap.__init__(self, cell_size, grid_x_length, grid_y_length)
        self._robot_x_length = robot_x_length
        self._robot_y_length = robot_y_length
        self._n_orientations = n_orientations

        self._grid_map = np.zeros((
            int(self.grid_x_length/self.cell_size),
            int(self.grid_y_length/self.cell_size),
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

        i_robot_orientation = 0 # this should loop through orientations

        for (key, obj) in objects.items():

            match obj.type:
                case "unmovable":
                    # TODO 1.0 is not the orientation which I acually want here
                    self.setup_object(obj, 1.0, i_robot_orientation, 1)

                case "movable":
                    self.setup_object(obj, 1.0, i_robot_orientation, 2)

                case "unknown":
                        self.setup_object(obj, 0, i_robot_orientation, 3)
                case _:
                    raise TypeError(f"unknown type: {obj.type}")

    def setup_object(self, obj: Object, robot_orientation: float, i_robot_orientation: int, val: int):
        """ Set the object overlapping with grid cells to a integer value. """ 

        match obj.obstacle.type():

            case "cylinder"| "sphere":
                
                self.setup_circular_object(obj, robot_orientation, i_robot_orientation, val)

            case "box":
                print("TODO: create setup rectangular objectC")

            case _:
                raise TypeError(f"Could not recognise obstacle type: {obj.obstacle.type()}")

      
    def setup_circular_object(self, obj: Object, robot_orientation: float, i_robot_orientation: int, val: int):
        """ Set the circular object overlapping with grid cells to a integer value. """ 
         # find corner points of the robot
        # TODO: filter some points pretty close ot the obstacle, so you do not to loop over all of them
        # assuming that is faster than calculating every line to every cylinder or edge of obstacle
        cos_rw = math.cos(robot_orientation)*self.robot_x_length/2
        sin_rw = math.sin(robot_orientation)*self.robot_x_length/2
        cos_rl = math.cos(robot_orientation)*self.robot_y_length/2
        sin_rl = math.sin(robot_orientation)*self.robot_y_length/2
        
        obj_xy = obj.state.get_xy_position()
        max_robot_obj_distance= max(self.robot_y_length, self.robot_x_length)/2 + obj.obstacle.radius()

        (obj_clearance_x_min, _) = self.position_to_cell_idx(obj_xy[0]-max_robot_obj_distance, 0) 
        (obj_clearance_x_max, _) = self.position_to_cell_idx(obj_xy[0]+max_robot_obj_distance, 0) 
        (_, obj_clearance_y_min) = self.position_to_cell_idx(0, obj_xy[1]-max_robot_obj_distance) 
        (_, obj_clearance_y_max) = self.position_to_cell_idx(0, obj_xy[1]+max_robot_obj_distance) 


        # find the boundaries imporatnt to check, do not loop over everything
        for x_idx in range(obj_clearance_x_min, obj_clearance_x_max+1):
            for y_idx in range(obj_clearance_y_min, obj_clearance_y_max+1):
                # if the cell is inside the object, it is in collision
                if np.linalg.norm(self.cell_idx_to_position(x_idx, y_idx)-obj_xy) <= obj.obstacle.radius():
                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val
                    continue

                # robot corner points a, b, c and d
                a = self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([-sin_rl+cos_rw, cos_rl+sin_rw])

                b =  self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([-sin_rl-cos_rw, cos_rl-sin_rw])

                c =  self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([+sin_rl-cos_rw, -cos_rl+sin_rw])

                d =  self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([+sin_rl+cos_rw, -cos_rl-sin_rw])

                obj_r = obj.obstacle.radius() 


                # check if the edges of the robot overlap with the obstacle
                if self.distance_point_to_line(obj_xy, a, b) <= obj_r:
                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val
                    continue

                elif self.distance_point_to_line(obj_xy, b, c) <= obj_r:
                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val
                    continue

                elif self.distance_point_to_line(obj_xy, c, d) <= obj_r:
                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val
                    continue

                elif self.distance_point_to_line(obj_xy, d, a) <= obj_r:
                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val
                    continue


    def distance_point_to_line(self, p: np.ndarray, lp1: np.ndarray, lp2: np.ndarray) -> float:
        """ returns the minimal distance from a line to a point. 
        if the normal distance is not on the line, the distance to
        minimal distance to the edge of the line is returned.
        """
        assert p.shape == (2,), "points has an incorrect shape"
        assert lp1.shape == (2,) and lp2.shape == (2,), "line points have incorrect shape"

        dp = lp2 - lp1
        st = dp[0]**2 + dp[1]**2
        u = ((p[0] - lp1[0]) * dp[0] + (p[1] - lp1[1]) * dp[1]) / st

        if u > 1.: 
            u = 1.
        if u < 0.:
            u = 0.

        dx = (lp1[0] + u * dp[0]) - p[0]
        dy = (lp1[1] + u * dp[1]) - p[1]

        return np.sqrt(dx**2 + dy**2)

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

    def occupancy(self, x_idx: int, y_idx: int, *args):
        if (x_idx > self.grid_map.shape[0] or
            x_idx < 0):
            raise ValueError(f"x_idx should be in range [0, {self.grid_map.shape[0]}] and is {x_idx}")

        if (y_idx > self.grid_map.shape[1] or
            y_idx < 0):
            raise ValueError(f"y_idx should be in range [0, {self.grid_map.shape[1]}] and is {y_idx}")

        if len(args) != 1: 
            raise TypeError(f"*args must be an integer")

        if not (isinstance(args[0], int) and
            type(args[0]) is int):
            raise TypeError(f"*args must be an integer")

        i_orientation = int(args[0])

        if (i_orientation > self.n_orientations or
            i_orientation < 0):
            raise ValueError(f"*args must be an integer in range [0, {self.n_orientations}] and is {i_orientation}")


        return self.grid_map[y_idx, x_idx, i_orientation]

    def visualise(self, i_orientation, objects):
        """ Display the occupancy map for a specific orientation of the robot. """

        fig = px.imshow(self.grid_map[:,:,i_orientation],
                y=list(np.arange((self.cell_size-self.grid_x_length)/2, (self.cell_size+self.grid_x_length)/2, self.cell_size)),
                x=list(np.arange((self.cell_size-self.grid_y_length)/2, (self.cell_size+self.grid_y_length)/2, self.cell_size)),

                labels={"x": "y-axis", "y": "x-axis"},
                title="Occupancy Map for Rectangular Robot",
                )

        # add the objects over the gridmap
        for obj in objects.values():
            match obj.obstacle.type():
                case "sphere":
                    fig.add_shape(type="circle",
                            xref="x", yref="y",
                            fillcolor="rgba(26,150,65,0.5)",
                            # review that please
                            x0=obj.state.pos[1]-obj.obstacle.radius(),
                            y0=obj.state.pos[0]-obj.obstacle.radius(),
                            x1=obj.state.pos[1]+obj.obstacle.radius(),
                            y1=obj.state.pos[0]+obj.obstacle.radius(),
                            line_color="LawnGreen",
                            )

                case "cylinder":
                    fig.add_shape(type="circle",
                            xref="x", yref="y",
                            fillcolor="rgba(26,150,65,0.5)",
                            # review that please
                            x0=obj.state.pos[1]-obj.obstacle.radius(),
                            y0=obj.state.pos[0]-obj.obstacle.radius(),
                            x1=obj.state.pos[1]+obj.obstacle.radius(),
                            y1=obj.state.pos[0]+obj.obstacle.radius(),
                            line_color="LightSeaGreen",
                            )
                case "box": 
                    # TODO: what to do if the robot is tilted, under an angle?
                    fig.add_shape(type="rect",
                            xref="x", yref="y",
                            fillcolor="rgba(26,150,65,0.5)",
                            # review that please
                            x0=obj.state.pos[1]-obj.obstacle.length()/2,
                            y0=obj.state.pos[0]-obj.obstacle.width()/2,
                            x1=obj.state.pos[1]+obj.obstacle.length()/2,
                            y1=obj.state.pos[0]+obj.obstacle.width()/2,
                            line_color="ForestGreen",
                            )
                case _:
                    raise TypeError(f"Could not recognise obstacle type: {obj.obstacle.type()}")

        fig.show()

    @property
    def grid_map(self):
        return self._grid_map

    @property
    def robot_x_length(self):
        return self._robot_x_length

    @property
    def robot_y_length(self):
        return self._robot_y_length

    @property
    def n_orientations(self):
        return self._n_orientations
