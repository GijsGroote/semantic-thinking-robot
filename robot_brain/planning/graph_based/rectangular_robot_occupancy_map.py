import sys
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
        robot_position: np.ndarray,
        n_orientations: int,
        robot_x_length: float,
        robot_y_length: float):

        OccupancyMap.__init__(self, cell_size, grid_x_length, grid_y_length, objects, robot_position, n_orientations)
        self._robot_x_length = robot_x_length
        self._robot_y_length = robot_y_length

        self._grid_map = np.zeros((
            int(self.grid_x_length/self.cell_size),
            int(self.grid_y_length/self.cell_size),
            n_orientations))

      
    def setup_circular_object(self, obj: Object, val: int, r_orien: float, i_r_orien: int):
        """ Set the circular object overlapping with grid cells (representing the robot) to a integer value. """ 

        # cos_rl = cos(orientation_of_robot) * robot_length_in_x / 2
        cos_rl = math.cos(r_orien)*self.robot_y_length/2
        sin_rl = math.sin(r_orien)*self.robot_y_length/2
        cos_rw = math.cos(r_orien)*self.robot_x_length/2
        sin_rw = math.sin(r_orien)*self.robot_x_length/2
        
        obj_xy = obj.state.get_xy_position()
        max_robot_obj_distance= (math.sqrt(self.robot_x_length**2 + self.robot_y_length**2))/2 + obj.obstacle.radius()

        # only search around obstacle 
        (obj_clearance_x_min, obj_clearance_y_min) = self.position_to_cell_idx_or_grid_edge(obj_xy[0]-max_robot_obj_distance, obj_xy[1]-max_robot_obj_distance)
        (obj_clearance_x_max, obj_clearance_y_max) = self.position_to_cell_idx_or_grid_edge(obj_xy[0]+max_robot_obj_distance, obj_xy[1]+max_robot_obj_distance)

        for x_idx in range(obj_clearance_x_min, obj_clearance_x_max+1):
            for y_idx in range(obj_clearance_y_min, obj_clearance_y_max+1):
                #  closeby (<= radius + smallest dimension robot) cells are always in collision with the obstacle 
                if np.linalg.norm(self.cell_idx_to_position(x_idx, y_idx)-obj_xy) <= obj.obstacle.radius() + min(self.robot_x_length, self.robot_y_length) / 2:
                    self.grid_map[x_idx, y_idx, i_r_orien] = val
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
                    self.grid_map[x_idx, y_idx, i_r_orien] = val
                    continue

                elif minimal_distance_point_to_line(obj_xy, b, c) <= obj_r:
                    self.grid_map[x_idx, y_idx, i_r_orien] = val
                    continue

                elif minimal_distance_point_to_line(obj_xy, c, d) <= obj_r:
                    self.grid_map[x_idx, y_idx, i_r_orien] = val
                    continue

                elif minimal_distance_point_to_line(obj_xy, d, a) <= obj_r:
                    self.grid_map[x_idx, y_idx, i_r_orien] = val
                    continue

    def setup_rectangular_object(self, obj: object, val: int, r_orien: float, i_r_orien: int):
        """ set the rectangular object overlapping with grid cells (representing the robot) to a integer value. """ 
        
        # TODO: project a box object toward the ground plane, find the edge points and convert that area to 
        # obstacle space or some other space, Gijs Groote 4 oct 2022.

        cos_rl = math.cos(r_orien)*self.robot_y_length/2
        sin_rl = math.sin(r_orien)*self.robot_y_length/2
        cos_rw = math.cos(r_orien)*self.robot_x_length/2
        sin_rw = math.sin(r_orien)*self.robot_x_length/2

      
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
                    self.grid_map[x_idx, y_idx, i_r_orien] = val 
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

                    self.grid_map[x_idx, y_idx, i_r_orien] = val 

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
    
    def shortest_path(self, pose_2d_start: np.ndarray, pose_2d_target:np.ndarray) -> list:
        """ use the Dijkstra algorithm to find the shortest path. """ 

        # convert position to indices on the grid
        idx_start = (x_idx_start, y_idx_start, orien_idx_start) = self.pose_2d_to_cell_idx(pose_2d_start)
        idx_target = (x_idx_target, y_idx_target, orien_idx_target) = self.pose_2d_to_cell_idx(pose_2d_target)
        
        #TODO: split visited and previous apart
        # vis_prev holds for a cell 2d index the following information:
        # a visited flag (0 for unvisited, 1 for in the queue, 2 for visited) in vis_prev[x_idx, y_idx, orien_idx, 0]
        # the previous cell pose in vis_prev[x_idx, y_idx, orien_idx, 1:4]
        vis_prev = np.zeros((self.grid_map.shape[0], self.grid_map.shape[1], self.grid_map.shape[2], 4)).astype(int)
        
        # set all cost to maximal size, except for the starting cell position
        cost = sys.maxsize*np.ones(self.grid_map.shape)
        cost[x_idx_start, y_idx_start, orien_idx_start] = 0
        
        queue = []
        queue.append(idx_start)
    
        (x_max, y_max, orien_max) = self.grid_map.shape
        
        while len(queue) != 0:
            cell_pose_temp = queue.pop(0)
            
            # set cell_pose_temp to visited
            vis_prev[cell_pose_temp[0], cell_pose_temp[1], cell_pose_temp[2], 0] = 2

            x_low = max(cell_pose_temp[0]-1, 0)
            x_high = min(cell_pose_temp[0]+1, x_max-1)
            y_low = max(cell_pose_temp[1]-1, 0)
            y_high = min(cell_pose_temp[1]+1, y_max-1)
            orien_low = max(cell_pose_temp[2]-1, 0)
            orien_high = min(cell_pose_temp[2]+1, orien_max-1)

            # loop though neighboring indexes
            for x_idx in range(x_low, x_high+1):
                for y_idx in range(y_low, y_high+1):
                    for orien_idx in range(orien_low, orien_high+1):

                        idx = (x_idx, y_idx, orien_idx)

                        # only compare unvisited cells
                        if vis_prev[idx + (0,)] != 2:

                            # put cell in the queue if not already in there
                            if vis_prev[idx + (0,)] == 0:
                                vis_prev[idx + (0,)] = 1 
                                queue.append(idx)
                            
                            temp_cost = cost[cell_pose_temp] + np.linalg.norm(cell_pose_temp-np.array(idx))
                            if temp_cost < cost[idx]:

                                cost[idx] = temp_cost
                                vis_prev[x_idx, y_idx, orien_idx, 1:4] = cell_pose_temp
                                



        shortest_path_reversed = []
        cell_pose_temp = idx_target
        # cell_pose_temp = np.array([x_idx_start, y_idx_start, orien_idx_start])
        
        # find shortest path from target to start
        while not all(x == y for x, y in zip(cell_pose_temp, idx_start)):

            shortest_path_reversed.append(vis_prev[cell_pose_temp[0],cell_pose_temp[1],cell_pose_temp[2], 1:4])
            cell_pose_temp = vis_prev[cell_pose_temp[0],cell_pose_temp[1],cell_pose_temp[2], 1:4]

        # reverse list and convert to 2D positions
        shortest_path_reversed.pop()
        shortest_path = [tuple(pose_2d_start)]

        while len(shortest_path_reversed) != 0:
            shortest_path.append(self.cell_idx_to_pose_2d(*shortest_path_reversed.pop()))
        shortest_path.append(tuple(pose_2d_target))
        return shortest_path


    def pose_2d_to_cell_idx(self, pose_2d: np.ndarray) -> (int, int, int):
        """ returns the index of the cell a 2D pose (x_position, y_position, orientation)
        raises an error if the 2D pose is outside of the grid
        """
        if pose_2d.shape != (3,):
            raise IndexError(f"the shape of pose_2d is {pose_2d.shape} and should be (3,)")
        if pose_2d[2] >= 2*math.pi or pose_2d[2] < 0:
            raise ValueError(f"orientation: {pose_2d[2]} is not in the interval [0, 2*pi)")
        
        normalised_orien= pose_2d[2]/(2*math.pi) + 1/(self.n_orientations*2)
        if normalised_orien >= 1:
            normalised_orien = 0

        orien_idx = int(normalised_orien*self.n_orientations)
        
        (x_idx, y_idx) = self.position_to_cell_idx(pose_2d[0], pose_2d[1])

        return (x_idx, y_idx, orien_idx)
    
    def cell_idx_to_pose_2d(self, x_idx: int, y_idx: int, orien_idx: int) -> (float, float, float):
        """ returns the center position that the cell represents. """

        if (x_idx >= self.grid_x_length/self.cell_size or
                x_idx < 0):
            raise IndexError(f"x_idx: {x_idx} is larger than the grid"\
                    f" [0, {int(self.grid_x_length/self.cell_size-1)}]")
        if (abs(y_idx) >= self.grid_y_length/self.cell_size or
                y_idx < 0):
            raise IndexError(f"y_idx: {y_idx} is larger than the grid"\
                    f" [0, {int(self.grid_y_length/self.cell_size-1)}]")
        if orien_idx < 0 or orien_idx >= self.n_orientations:
            raise IndexError(f"orien_idx: {orien_idx} is larger than the number of grid orientations"\
                    f" [0, {self.n_orientations-1}]")
        
        return (self.cell_size*(0.5+x_idx) - self.grid_x_length/2,
                self.cell_size*(0.5+y_idx) - self.grid_y_length/2,
                2*math.pi*orien_idx/self.n_orientations)

    

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
        r_orien = i_orientation/self.n_orientations*2*math.pi
        cos_rl = math.cos(r_orien)*self.robot_y_length/2
        sin_rl = math.sin(r_orien)*self.robot_y_length/2
        cos_rw = math.cos(r_orien)*self.robot_x_length/2
        sin_rw = math.sin(r_orien)*self.robot_x_length/2
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

