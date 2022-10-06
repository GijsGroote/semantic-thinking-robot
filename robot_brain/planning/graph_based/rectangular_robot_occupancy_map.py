
import numpy as np
import pickle
from robot_brain.planning.graph_based.occupancy_map import OccupancyMap
import math
import plotly.express as px
import plotly.graph_objects as go
import plotly.subplots as sp
import warnings

from helper_functions.geometrics import minimal_distance_point_to_line, point_in_rectangle, do_intersect
from robot_brain.planning.object import Object
from robot_brain.global_variables import FIG_BG_COLOR

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
        n_orientations: int,
        robot_position: np.ndarray=np.array([0,0,0])):

        OccupancyMap.__init__(self, cell_size, grid_x_length, grid_y_length, objects)
        self._robot_x_length = robot_x_length
        self._robot_y_length = robot_y_length
        self._robot_position = robot_position 
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
            case "cylinder":
                # "objects" x-axis is parallel to the global z-axis (normal situation)
                if not (math.isclose(math.sin(obj.state.ang_p[0]), 0, abs_tol=0.01) and 
                        math.isclose(math.sin(obj.state.ang_p[1]), 0, abs_tol=0.01)):
                    warnings.warn(f"obstacle {obj.name} is not in correct orientation (up/down is not up)")

                self.setup_circular_object(obj, robot_orientation, i_robot_orientation, val)

            case "sphere":
                self.setup_circular_object(obj, robot_orientation, i_robot_orientation, val)

            case "box":
                # "objects" x-axis is parallel to the global z-axis (normal situation)
                if not ((math.isclose(obj.state.ang_p[0], 0, abs_tol=0.01) or 
                        math.isclose(obj.state.ang_p[0], 2*math.pi, abs_tol=0.01)) and
                        math.isclose(obj.state.ang_p[1], 0, abs_tol=0.01) or
                        math.isclose(obj.state.ang_p[1], 2*math.pi, abs_tol=0.01)):
                    warnings.warn(f"obstacle {obj.name} is not in correct orientation (up is not up)")

                self.setup_rectangular_object(obj, robot_orientation, i_robot_orientation, val)
            case "urdf":
                warnings.warn(f"the urdf type is not yet implemented")

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
                if np.linalg.norm(self.cell_idx_to_position(x_idx, y_idx)-obj_xy) <= obj.obstacle.radius() + min(self.robot_x_length, self.robot_y_length) / 2:
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
        
        # TODO: project a box object toward the ground plane, find the edge points and convert that area to 
        # obstacle space or some other space, Gijs Groote 4 oct 2022.

        cos_rl = math.cos(robot_orientation)*self.robot_y_length/2
        sin_rl = math.sin(robot_orientation)*self.robot_y_length/2
        cos_rw = math.cos(robot_orientation)*self.robot_x_length/2
        sin_rw = math.sin(robot_orientation)*self.robot_x_length/2

      
        cos_ol = math.cos(obj.state.ang_p[2])*obj.obstacle.width()/2
        sin_ol = math.sin(obj.state.ang_p[2])*obj.obstacle.width()/2
        cos_ow = math.cos(obj.state.ang_p[2])*obj.obstacle.length()/2
        sin_ow = math.sin(obj.state.ang_p[2])*obj.obstacle.length()/2
        
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

    def visualise(self, i_orientation:int=0, save:bool=True):
        """ Display the occupancy map for a specific orientation of the robot. """
       
        grid_2D = np.reshape(self.grid_map[:,:,i_orientation], (int(self.grid_x_length/self.cell_size), int(self.grid_y_length/self.cell_size)))
        
        trace = go.Heatmap(
                x=list(np.arange((self.cell_size-(self.grid_y_length))/2, (self.cell_size+(self.grid_y_length))/2, self.cell_size)),
                y=list(np.arange((self.cell_size-(self.grid_x_length))/2, (self.cell_size+(self.grid_x_length))/2, self.cell_size)),
                z=grid_2D,
                type = "heatmap",
                colorscale = [[0.0, "rgba(11,156,49,0.1)"], [0.25, "rgba(11,156,49,0.1)"], [0.25, "#b2b2ff"], [0.5, "#b2b2ff"],
                    [0.5, "#e763fa"], [0.75, "#e763fa"], [0.75, "#ab63fa"], [1.0, "#ab63fa"]], 
                colorbar=dict(title="Space Legend",
                    tickvals=[0.3, 1.05, 1.8, 2.55, 3.3],
                    ticktext=["Free", "Obstacle", "Movable", "Unknown"],
                    y=.6,
                    len=.2,
                    ),
                )

        fig = go.Figure(data=trace)

        # put the robot on the map 
        robot_orientation = i_orientation/self.n_orientations*2*math.pi
        cos_rl = math.cos(robot_orientation)*self.robot_y_length/2
        sin_rl = math.sin(robot_orientation)*self.robot_y_length/2
        cos_rw = math.cos(robot_orientation)*self.robot_x_length/2
        sin_rw = math.sin(robot_orientation)*self.robot_x_length/2
        fig.add_trace(go.Scatter(
            x=[self.robot_position[1]],
            y=[self.robot_position[0]],
            text="robot",
            mode="text",
            textfont=dict(
                color="black",
                size=18,
                family="Arail",
                )
            )
        )

        fig.add_trace(go.Scatter(
            y=[self.robot_position[0]-sin_rl+cos_rw,
            self.robot_position[0]-sin_rl-cos_rw,
            self.robot_position[0]+sin_rl-cos_rw,
            self.robot_position[0]+sin_rl+cos_rw,
            self.robot_position[0]-sin_rl+cos_rw],
            x=[self.robot_position[1]+cos_rl+sin_rw,
                self.robot_position[1]+cos_rl-sin_rw,
                self.robot_position[1]-cos_rl-sin_rw,
                self.robot_position[1]-cos_rl+sin_rw,
                self.robot_position[1]+cos_rl+sin_rw],
            line_color="black",
            mode='lines'
            )
        )

        # add the boundaries of the map
        fig.add_shape(type="rect",
                x0=self.grid_y_length/2, y0=self.grid_x_length/2, x1=-self.grid_y_length/2, y1=-self.grid_x_length/2,
                line_color="black",
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
                    cos_ol = math.cos(obj.state.ang_p[2])*obj.obstacle.width()/2
                    sin_ol = math.sin(obj.state.ang_p[2])*obj.obstacle.width()/2
                    cos_ow = math.cos(obj.state.ang_p[2])*obj.obstacle.length()/2
                    sin_ow = math.sin(obj.state.ang_p[2])*obj.obstacle.length()/2

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

                case "urdf":
                    warnings.warn("the urdf objstacle is not yet implemented")

                case _:
                    raise TypeError(f"Could not recognise obstacle type: {obj.obstacle.type()}")

        fig.update_layout(
                height=900,
                showlegend= False,
                paper_bgcolor=FIG_BG_COLOR,
                plot_bgcolor=FIG_BG_COLOR,
            )

        fig.update_yaxes(autorange="reversed")
        if save:
            with open("/home/gijs/Documents/semantic-thinking-robot/robot_brain/dashboard/data/occupancy_map.pickle", "wb") as file:
                pickle.dump(fig, file)
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
    def n_orientations(self):
        return self._n_orientations

    @property
    def robot_position(self):
        return self._robot_position
