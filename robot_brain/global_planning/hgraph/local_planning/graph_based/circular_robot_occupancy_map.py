import numpy as np

import sys
from robot_brain.global_planning.hgraph.local_planning.graph_based.occupancy_map import OccupancyMap
import math
import plotly.graph_objects as go 
import warnings

import pickle

from helper_functions.geometrics import minimal_distance_point_to_line
from robot_brain.object import Object
from robot_brain.global_variables import FIG_BG_COLOR

class CircleRobotOccupancyMap(OccupancyMap):
    """ 2-dimensional occupancy map representing the environment
    in obstacle space, free space, movable obstacle space and
    unknown obstacle space for a circular robot.
    """
    def __init__(self,
            cell_size: float,
            grid_x_length: float,
            grid_y_length: float,
            objects: dict,
            robot_radius: float,
            robot_position: np.ndarray):

        OccupancyMap.__init__(self, cell_size, grid_x_length, grid_y_length, objects, robot_position, 1)
        self._robot_radius = robot_radius
        self._grid_map = np.zeros((
            int(self.grid_x_length/self.cell_size),
            int(self.grid_y_length/self.cell_size)))
       
    def setup_circular_object(self, obj: Object, val: int, r_orien: float, i_r_orien: int):
        """ Set the circular object overlapping with grid cells (representing the robot) to a integer value. """ 
        
        obj_xy = obj.state.get_xy_position()

        # only search around obstacle 
        (obj_clearance_x_min, obj_clearance_y_min) = self.position_to_cell_idx_or_grid_edge(
                obj_xy[0]-(self.robot_radius + obj.obstacle.radius()), 
                obj_xy[1]-(self.robot_radius + obj.obstacle.radius()))

        (obj_clearance_x_max, obj_clearance_y_max) = self.position_to_cell_idx_or_grid_edge(
                obj_xy[0]+(self.robot_radius + obj.obstacle.radius()),
                obj_xy[1]+(self.robot_radius + obj.obstacle.radius()))

        for x_idx in range(obj_clearance_x_min, obj_clearance_x_max+1):
            for y_idx in range(obj_clearance_y_min, obj_clearance_y_max+1):
                #  closeby (<= radius + smallest dimension robot) cells are always in collision with the obstacle 
                if np.linalg.norm(self.cell_idx_to_position(x_idx, y_idx)-obj_xy) <= obj.obstacle.radius() + self.robot_radius:
                    self.grid_map[x_idx, y_idx] = val

    def setup_rectangular_object(self, obj: object, val: int, r_orien: float, i_r_orien: int):
        """ set the rectangular object overlapping with grid cells (representing the robot) to a integer value. """ 

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

        max_robot_to_obj_x_distance = self.robot_radius + abs(sin_ol) + abs(cos_ow)
        max_robot_to_obj_y_distance = self.robot_radius + abs(cos_ol) + abs(sin_ow)
        
        obj_xy = obj.state.get_xy_position()

        # only search around obstacle
        (obj_clearance_x_min, obj_clearance_y_min) = self.position_to_cell_idx_or_grid_edge(
                obj_xy[0]-max_robot_to_obj_x_distance, obj_xy[1]-max_robot_to_obj_y_distance)

        (obj_clearance_x_max, obj_clearance_y_max) = self.position_to_cell_idx_or_grid_edge(
                obj_xy[0]+max_robot_to_obj_x_distance, obj_xy[1]+max_robot_to_obj_y_distance)

        for x_idx in range(obj_clearance_x_min, obj_clearance_x_max+1):
            for y_idx in range(obj_clearance_y_min, obj_clearance_y_max+1):

                #  closeby (<= robot_radius + smallest dimension obstacle) cells are always in collision with the obstacle 
                if (np.linalg.norm(self.cell_idx_to_position(x_idx, y_idx)-obj_xy) <= self.robot_radius
                    + min(obj.obstacle.width(), obj.obstacle.length()) / 2):

                    self.grid_map[x_idx, y_idx] = val
                    continue

                robot_xy = np.array(self.cell_idx_to_position(x_idx, y_idx))
                
                # check if the edges of the robot overlap with the obstacle
                if minimal_distance_point_to_line(robot_xy, obst_a, obst_b) <= self.robot_radius:
                    self.grid_map[x_idx, y_idx] = val
                    continue

                elif minimal_distance_point_to_line(robot_xy, obst_b, obst_c) <= self.robot_radius:
                    self.grid_map[x_idx, y_idx] = val
                    continue

                elif minimal_distance_point_to_line(robot_xy, obst_c, obst_d) <= self.robot_radius:
                    self.grid_map[x_idx, y_idx] = val
                    continue

                elif minimal_distance_point_to_line(robot_xy, obst_d, obst_a) <= self.robot_radius:
                    self.grid_map[x_idx, y_idx] = val
                    continue

    def occupancy(self, position: np.ndarray) -> int:
        """ returns the occupancy of the grid cell """
        assert position.shape == (2,), f"position not of shape (2,) but {position.shape}"
        idx = self.position_to_cell_idx(position[0], position[1]) 
        return self.idx_to_occupancy(*idx)


    def idx_to_occupancy(self, x_idx: int, y_idx: int):
        if (x_idx > self.grid_map.shape[0] or
            x_idx < 0):
            raise ValueError(f"x_idx should be in range [0, {self.grid_map.shape[0]}] and is {x_idx}")

        if (y_idx > self.grid_map.shape[1] or
            y_idx < 0):
            raise ValueError(f"y_idx should be in range [0, {self.grid_map.shape[1]}] and is {y_idx}")

        return self.grid_map[x_idx, y_idx]
    
    def shortest_path(self, position_start: np.ndarray, position_target: np.ndarray) -> list:

        # convert position to indices on the grid
        idx_start = (x_idx_start, y_idx_start) = self.position_to_cell_idx(position_start[0], position_start[1])
        idx_target = (x_idx_target, y_idx_target) = self.position_to_cell_idx(position_target[0], position_target[1])

        # a visited flag (0 for unvisited, 1 for in the queue, 2 for visited)
        visited = np.zeros((self.grid_map.shape[0], self.grid_map.shape[1])).astype(int)
        previous_cell = np.zeros((self.grid_map.shape[0], self.grid_map.shape[1], 2)).astype(int)
        
        # set all cost to maximal size, except for the starting cell position
        cost = sys.maxsize*np.ones(self.grid_map.shape)
        cost[x_idx_start, y_idx_start] = 0
        
        queue = []
        queue.append(idx_start)
    
        (x_max, y_max) = self.grid_map.shape
        
        while len(queue) != 0:
            cell_pose_temp = queue.pop(0)
            
            # set cell_pose_temp to visited
            visited[cell_pose_temp[0], cell_pose_temp[1]] = 2

            x_low = max(cell_pose_temp[0]-1, 0)
            x_high = min(cell_pose_temp[0]+1, x_max-1)
            y_low = max(cell_pose_temp[1]-1, 0)
            y_high = min(cell_pose_temp[1]+1, y_max-1)

            # loop though neighboring indexes
            for x_idx in range(x_low, x_high+1):
                for y_idx in range(y_low, y_high+1):

                        idx = (x_idx, y_idx)

                        # only compare unvisited cells
                        if visited[idx] != 2:

                            # path cannot go through obstacles
                            if self.idx_to_occupancy(x_idx, y_idx) != 1:

                                # put cell in the queue if not already in there
                                if visited[idx] == 0:
                                    visited[idx] = 1 
                                    queue.append(idx)
                                
                                temp_cost = cost[cell_pose_temp] + np.linalg.norm(cell_pose_temp-np.array(idx))
                                if temp_cost < cost[idx]:

                                    cost[idx] = temp_cost
                                    previous_cell[idx[0], idx[1], :] = cell_pose_temp

        shortest_path_reversed = []
        cell_pose_temp = idx_target
       
        # find shortest path from target to start
        while not all(x == y for x, y in zip(cell_pose_temp, idx_start)):

            shortest_path_reversed.append(previous_cell[cell_pose_temp[0], cell_pose_temp[1], :])
            cell_pose_temp = previous_cell[cell_pose_temp[0], cell_pose_temp[1], :]

        # reverse list and convert to 2D positions
        if len(shortest_path_reversed) != 0:
            shortest_path_reversed.pop()

        shortest_path = [tuple(position_start)]

        while len(shortest_path_reversed) != 0:
            # CELL_IDX_TO_PSE_2D SHOULD NOT BE USED HERE, USE THE 2 DIM VERSION
            shortest_path.append(self.cell_idx_to_position(*shortest_path_reversed.pop()))
        shortest_path.append(tuple(position_target))

        return shortest_path



    def visualise(self, save:bool=True):
        """ Display the occupancy map for a specific orientation of the robot. """
       
        
        trace = go.Heatmap(
                x=list(np.arange((self.cell_size-(self.grid_y_length))/2, (self.cell_size+(self.grid_y_length))/2, self.cell_size)),
                y=list(np.arange((self.cell_size-(self.grid_x_length))/2, (self.cell_size+(self.grid_x_length))/2, self.cell_size)),
                z=self.grid_map,
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
        fig.add_shape(type="circle",
                xref="x", yref="y",
                x0=self.robot_position[1]-self.robot_radius,
                y0=self.robot_position[0]-self.robot_radius,
                x1=self.robot_position[1]+self.robot_radius,
                y1=self.robot_position[0]+self.robot_radius,
                line_color="black",
        )

        # add the boundaries of the map
        fig.add_shape(type="rect",
                x0=self.grid_y_length/2, y0=self.grid_x_length/2, x1=-self.grid_y_length/2, y1=-self.grid_x_length/2,
                line_color="black",
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
            with open("/home/gijs/Documents/semantic-thinking-robot/dashboard/data/occupancy_map.pickle", "wb") as file:
                pickle.dump(fig, file)
        else:
            fig.show()


    @property
    def grid_map(self):
        return self._grid_map

    @property
    def robot_radius(self):
        return self._robot_radius
