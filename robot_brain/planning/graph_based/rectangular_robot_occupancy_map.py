import numpy as np
import pickle
from robot_brain.planning.graph_based.occupancy_map import OccupancyMap
import math
import plotly.express as px
import plotly.graph_objects as go

from helper_functions.geometrics import minimal_distance_point_to_line, point_in_rectangle, do_intersect
from robot_brain.planning.object import Object


from scipy.spatial.transform import Rotation

class RectangularRobotOccupancyMap(OccupancyMap):
    """ Occupancy map represents the environment in obstacle space
    free space, movable obstacle space and unknown obstacle space.
    """

    def __init__(self,
        cell_size: float,
        grid_x_length: float,
        grid_y_length: float,
        objects: dict,
        robot_x_length: float,
        robot_y_length: float,
        n_orientations: int):

        OccupancyMap.__init__(self, cell_size, grid_x_length, grid_y_length, objects)
        self._robot_x_length = robot_x_length
        self._robot_y_length = robot_y_length
        self._robot_dimension_min = min(self.robot_x_length, self.robot_y_length)
        self._robot_dimension_max = max(self.robot_x_length, self.robot_y_length)
        self._n_orientations = n_orientations

        self._grid_map = np.zeros((
            int(self.grid_x_length/self.cell_size),
            int(self.grid_y_length/self.cell_size),
            n_orientations))

    def setup(self):
        """
        Setup the gridmap for a number of obstacles.

        indices in the grid_map indicat the following:
            0 -> free space
            1 -> obstacle space
            2 -> movable object space
            3 -> unknown object space
        """
        for i_robot_orientation in range(self.n_orientations):
            for obj in self.objects.values():
                match obj.type:
                    case "unmovable":
                        self.setup_object(obj, 2*math.pi*i_robot_orientation/self.n_orientations, i_robot_orientation, 1)

                    case "movable":
                        self.setup_object(obj, 2*math.pi*i_robot_orientation/self.n_orientations, i_robot_orientation, 2)

                    case "unknown":
                        self.setup_object(obj, 2*math.pi*i_robot_orientation/self.n_orientations, i_robot_orientation, 3)
                    case _:
                        raise TypeError(f"unknown type: {obj.type}")

    def setup_object(self, obj: Object, robot_orientation: float, i_robot_orientation: int, val: int):
        """ Set the object overlapping with grid cells to a integer value. """ 
        
        match obj.obstacle.type():
            case "cylinder"| "sphere":
                self.setup_circular_object(obj, robot_orientation, i_robot_orientation, val)

            case "box":
                self.setup_rectangular_object(obj, robot_orientation, i_robot_orientation, val)

            case _:
                raise TypeError(f"Could not recognise obstacle type: {obj.obstacle.type()}")

      
    def setup_circular_object(self, obj: Object, robot_orientation: float, i_robot_orientation: int, val: int):
        """ Set the circular object overlapping with grid cells (representing the robot) to a integer value. """ 

        # cos_rl = cos(orientation_of_robot) * robot_length_in_x / 2
        cos_rl = math.cos(robot_orientation)*self.robot_y_length/2
        sin_rl = math.sin(robot_orientation)*self.robot_y_length/2
        cos_rw = math.cos(robot_orientation)*self.robot_x_length/2
        sin_rw = math.sin(robot_orientation)*self.robot_x_length/2
        
        obj_xy = obj.state.get_xy_position()
        max_robot_obj_distance= (math.sqrt(self.robot_x_length**2 + self.robot_y_length**2))/2 + obj.obstacle.radius()

        # only search around obstacle 
        (obj_clearance_x_min, obj_clearance_y_min) = self.position_to_cell_idx_or_grid_edge(obj_xy[0]-max_robot_obj_distance, obj_xy[1]-max_robot_obj_distance)
        (obj_clearance_x_max, obj_clearance_y_max) = self.position_to_cell_idx_or_grid_edge(obj_xy[0]+max_robot_obj_distance, obj_xy[1]+max_robot_obj_distance)

        for x_idx in range(obj_clearance_x_min, obj_clearance_x_max+1):
            for y_idx in range(obj_clearance_y_min, obj_clearance_y_max+1):
                #  closeby (<= radius + smallest dimension robot) cells are always in collision with the obstacle 
                if np.linalg.norm(self.cell_idx_to_position(x_idx, y_idx)-obj_xy) <= obj.obstacle.radius() + self.robot_dimension_min / 2:
                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val
                    continue

               # corner points of the robot
                a = self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([-sin_rl+cos_rw, cos_rl+sin_rw])

                b =  self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([-sin_rl-cos_rw, cos_rl-sin_rw])

                c =  self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([+sin_rl-cos_rw, -cos_rl-sin_rw])

                d =  self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([sin_rl+cos_rw, -cos_rl+sin_rw])

                obj_r = obj.obstacle.radius() 

                # check if the edges of the robot overlap with the obstacle
                if minimal_distance_point_to_line(obj_xy, a, b) <= obj_r:
                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val
                    continue

                elif minimal_distance_point_to_line(obj_xy, b, c) <= obj_r:
                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val
                    continue

                elif minimal_distance_point_to_line(obj_xy, c, d) <= obj_r:
                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val
                    continue

                elif minimal_distance_point_to_line(obj_xy, d, a) <= obj_r:
                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val
                    continue

    def setup_rectangular_object(self, obj: object, robot_orientation: float, i_robot_orientation: int, val: int):
        """ set the rectangular object overlapping with grid cells (representing the robot) to a integer value. """ 

        cos_rl = math.cos(robot_orientation)*self.robot_y_length/2
        sin_rl = math.sin(robot_orientation)*self.robot_y_length/2
        cos_rw = math.cos(robot_orientation)*self.robot_x_length/2
        sin_rw = math.sin(robot_orientation)*self.robot_x_length/2
        # cos_ol = cos(orientation_of_object)* object_length / 2
        cos_ol = math.cos(obj.state.ang_p[2])*obj.obstacle.length()/2
        sin_ol = math.sin(obj.state.ang_p[2])*obj.obstacle.length()/2
        cos_ow = math.cos(obj.state.ang_p[2])*obj.obstacle.width()/2
        sin_ow = math.sin(obj.state.ang_p[2])*obj.obstacle.width()/2
        # corner points of the obstacle
        obst_a = np.array([obj.state.pos[0]-sin_ol+cos_ow, obj.state.pos[1]+cos_ol+sin_ow])
        obst_b = np.array([obj.state.pos[0]-sin_ol-cos_ow, obj.state.pos[1]+cos_ol-sin_ow])
        obst_c = np.array([obj.state.pos[0]+sin_ol-cos_ow, obj.state.pos[1]-cos_ol-sin_ow])
        obst_d = np.array([obj.state.pos[0]+sin_ol+cos_ow, obj.state.pos[1]-cos_ol+sin_ow])

        max_robot_to_obj_x_distance = abs(sin_rl) + abs(cos_rw) + abs(sin_ol) + abs(cos_ow)
        max_robot_to_obj_y_distance = abs(cos_rl) + abs(sin_rw) + abs(cos_ol) + abs(sin_ow)
        
        obj_xy = obj.state.get_xy_position()

        # only search around obstacle
        (x_min, y_min) = self.position_to_cell_idx_or_grid_edge(obj_xy[0]-max_robot_to_obj_x_distance, obj_xy[1]-max_robot_to_obj_y_distance)
        (x_max, y_max) = self.position_to_cell_idx_or_grid_edge(obj_xy[0]+max_robot_to_obj_x_distance, obj_xy[1]+max_robot_to_obj_y_distance)
        
        # orientation with cos/sin could make x_min > x_max
        obj_clearance_x_min = min(x_min, x_max)
        obj_clearance_x_max = max(x_min, x_max)
        obj_clearance_y_min = min(y_min, y_max)
        obj_clearance_y_max = max(y_min, y_max)

        for x_idx in range(obj_clearance_x_min, obj_clearance_x_max+1):
            for y_idx in range(obj_clearance_y_min, obj_clearance_y_max+1):

                # if the center of the robot is in the obstacle, then they are in collision
                if point_in_rectangle(np.array(self.cell_idx_to_position(x_idx, y_idx)),
                        obst_a, obst_b, obst_c):
                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val 
                    continue

                # corner points of the robot
                a = self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([-sin_rl+cos_rw, cos_rl+sin_rw])

                b =  self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([-sin_rl-cos_rw, cos_rl-sin_rw])

                c =  self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([+sin_rl-cos_rw, -cos_rl-sin_rw])

                d =  self.cell_idx_to_position(x_idx, y_idx) \
                        + np.array([sin_rl+cos_rw, -cos_rl+sin_rw])

                # check if the edges of the robot overlap with the obstacle
                # TODO: this cannot every be efficient, replace with a more efficient rectangle overlapping with rectangle algorithm, Gijs 2 oct 2022.
                if (do_intersect(a, b, obst_a, obst_b) or
                    do_intersect(a, b, obst_b, obst_c) or
                    do_intersect(a, b, obst_c, obst_d) or
                    do_intersect(a, b, obst_d, obst_a) or

                    do_intersect(b, c, obst_a, obst_b) or
                    do_intersect(b, c, obst_b, obst_c) or
                    do_intersect(b, c, obst_c, obst_d) or
                    do_intersect(b, c, obst_d, obst_a) or

                    do_intersect(c, d, obst_a, obst_b) or
                    do_intersect(c, d, obst_b, obst_c) or
                    do_intersect(c, d, obst_c, obst_d) or
                    do_intersect(c, d, obst_d, obst_a) or

                    do_intersect(d, a, obst_a, obst_b) or
                    do_intersect(d, a, obst_b, obst_c) or
                    do_intersect(d, a, obst_c, obst_d) or
                    do_intersect(d, a, obst_d, obst_a)):

                    self.grid_map[x_idx, y_idx, i_robot_orientation] = val 



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

    def position_to_cell_idx_or_grid_edge(self, x_position: float, y_position: float) -> (int, int):
        """ returns the index of the cell a position is in, 
        if the position is outside the boundary of the grid map the edges 
        will be returned.
        """
        try: 
            x_idx = self.position_to_cell_idx(x_position, 0)[0]
        except IndexError as e:
            if x_position > self.grid_x_length/2:
                x_idx = int(self.grid_x_length/self.cell_size-1)
            elif x_position < -self.grid_x_length/2:
                x_idx = 0
            else:
                raise IndexError(f"x_position: {x_position} could not be converted to cell index.")

        try: 
            y_idx = self.position_to_cell_idx(0, y_position)[1]
        except IndexError as e:
            if y_position > self.grid_y_length/2:
                y_idx = int(self.grid_y_length/self.cell_size-1)
            elif y_position < -self.grid_y_length/2:
                y_idx = 0
            else:
                raise IndexError(f"y_position: {y_position} could not be converted to cell index.")

        return (x_idx, y_idx)

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

    def visualise(self, i_orientation=0, save=False):
        """ Display the occupancy map for a specific orientation of the robot. """

        fig = px.imshow(self.grid_map[:,:,i_orientation],
                x=list(np.arange((self.cell_size-self.grid_y_length)/2, (self.cell_size+self.grid_y_length)/2, self.cell_size)),
                y=list(np.arange((self.cell_size-self.grid_x_length)/2, (self.cell_size+self.grid_x_length)/2, self.cell_size)),
                labels={"x": "y-axis", "y": "x-axis"},
                )

        # add the objects over the gridmap
        for obj in self.objects.values():

            # add the name of the object
            fig.add_trace(go.Scatter(
                x=[obj.state.pos[1]],
                y=[obj.state.pos[0]],
                text=[obj.name],
                mode="text",
                textfont=dict(
                    color="black",
                    size=18,
                    family="Arail",
                    )
                )
                )

            match obj.obstacle.type():
                case "sphere" | "cylinder":
                    fig.add_shape(type="circle",
                            xref="x", yref="y",
                            x0=obj.state.pos[1]-obj.obstacle.radius(),
                            y0=obj.state.pos[0]-obj.obstacle.radius(),
                            x1=obj.state.pos[1]+obj.obstacle.radius(),
                            y1=obj.state.pos[0]+obj.obstacle.radius(),
                            line_color="black",
                            )
                case "box":
                    cos_ol = math.cos(obj.state.ang_p[2])*obj.obstacle.length()/2
                    sin_ol = math.sin(obj.state.ang_p[2])*obj.obstacle.length()/2
                    cos_ow = math.cos(obj.state.ang_p[2])*obj.obstacle.width()/2
                    sin_ow = math.sin(obj.state.ang_p[2])*obj.obstacle.width()/2

                    fig.add_trace(go.Scatter(y=[obj.state.pos[0]-sin_ol+cos_ow,
                        obj.state.pos[0]-sin_ol-cos_ow,
                        obj.state.pos[0]+sin_ol-cos_ow,
                        obj.state.pos[0]+sin_ol+cos_ow,
                        obj.state.pos[0]-sin_ol+cos_ow],
                        x=[obj.state.pos[1]+cos_ol+sin_ow,
                            obj.state.pos[1]+cos_ol-sin_ow,
                            obj.state.pos[1]-cos_ol-sin_ow,
                            obj.state.pos[1]-cos_ol+sin_ow,
                            obj.state.pos[1]+cos_ol+sin_ow],
                        line_color="black",
                        mode='lines'))

                case _:
                    raise TypeError(f"Could not recognise obstacle type: {obj.obstacle.type()}")

        fig.update_layout(
                paper_bgcolor= "rgb(230, 230, 255)",
                plot_bgcolor= "rgb(230, 230, 255)",
                coloraxis_showscale= False, 
                autosize = True,
                showlegend= False)

        if save:
            with open("/home/gijs/Documents/semantic-thinking-robot/robot_brain/dashboard/data/occupancy_map.pickle", "wb") as f:
                pickle.dump(fig, f)
        else:
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
    def robot_dimension_min(self):
        return self._robot_dimension_min

    @property
    def robot_dimension_max(self):
        return self._robot_dimension_max

    @property
    def n_orientations(self):
        return self._n_orientations

