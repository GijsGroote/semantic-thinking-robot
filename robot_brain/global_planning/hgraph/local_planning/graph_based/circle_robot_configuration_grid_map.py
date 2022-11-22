import numpy as np
import sys
from robot_brain.global_planning.hgraph.local_planning.graph_based.configuration_grid_map import ConfigurationGridMap 
import math
import plotly.graph_objects as go 
import warnings
import pickle

from helper_functions.geometrics import minimal_distance_point_to_line, point_in_rectangle
from robot_brain.obstacle import Obstacle
from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH

class CircleRobotConfigurationGridMap(ConfigurationGridMap):
    """ 2-dimensional configuration grid map representing the environment
    in obstacle space, free space, movable obstacle space and
    unknown obstacle space for a circular robot.
    """
    def __init__(self,
            cell_size: float,
            grid_x_length: float,
            grid_y_length: float,
            obstacles: dict,
            robot_cart_2d: np.ndarray,
            robot_radius: float):

        ConfigurationGridMap.__init__(self, cell_size, grid_x_length, grid_y_length, obstacles, robot_cart_2d, 1)
        self._robot_radius = robot_radius
        self._grid_map = np.zeros((
            int(self.grid_x_length/self.cell_size),
            int(self.grid_y_length/self.cell_size)))

    def setup_circle_obstacle(self, obst: Obstacle, val: int, r_orien: float, r_orien_idx: int):
        """ Set the circular obstacle overlapping with grid cells (representing the robot) to a integer value. """ 
        
        obst_cart_2d = obst.state.get_xy_position()

        # only search around obstacle 
        (obst_clearance_x_min, obst_clearance_y_min) = self.cart_2d_to_c_idx_or_grid_edge(
                obst_cart_2d[0]-(self.robot_radius + obst.properties.radius()), 
                obst_cart_2d[1]-(self.robot_radius + obst.properties.radius()))

        (obst_clearance_x_max, obst_clearance_y_max) = self.cart_2d_to_c_idx_or_grid_edge(
                obst_cart_2d[0]+(self.robot_radius + obst.properties.radius()),
                obst_cart_2d[1]+(self.robot_radius + obst.properties.radius()))

        for x_idx in range(obst_clearance_x_min, obst_clearance_x_max+1):
            for y_idx in range(obst_clearance_y_min, obst_clearance_y_max+1):
                #  closeby (<= radius + smallest dimension robot) cells are always in collision with the obstacle 
                if np.linalg.norm(self.c_idx_to_cart_2d(x_idx, y_idx)-obst_cart_2d) <= obst.properties.radius() + self.robot_radius:
                    self.grid_map[x_idx, y_idx] = val

    def setup_rectangular_obstacle(self, obst: Obstacle, val: int, r_orien: float, r_orien_idx: int):
        """ set the rectangular obstect overlapping with grid cells (representing the robot) to a integer value. """ 

        cos_ol = math.cos(obst.state.ang_p[2])*obst.properties.width()/2
        sin_ol = math.sin(obst.state.ang_p[2])*obst.properties.width()/2
        cos_ow = math.cos(obst.state.ang_p[2])*obst.properties.length()/2
        sin_ow = math.sin(obst.state.ang_p[2])*obst.properties.length()/2
        
        # corner points of the obstacle
        obst_a = np.array([obst.state.pos[0]-sin_ol+cos_ow, obst.state.pos[1]+cos_ol+sin_ow])
        obst_b = np.array([obst.state.pos[0]-sin_ol-cos_ow, obst.state.pos[1]+cos_ol-sin_ow])
        obst_c = np.array([obst.state.pos[0]+sin_ol-cos_ow, obst.state.pos[1]-cos_ol-sin_ow])
        obst_d = np.array([obst.state.pos[0]+sin_ol+cos_ow, obst.state.pos[1]-cos_ol+sin_ow])

        max_robot_to_obst_x_distance = self.robot_radius + abs(sin_ol) + abs(cos_ow)
        max_robot_to_obst_y_distance = self.robot_radius + abs(cos_ol) + abs(sin_ow)

        obst_cart_2d = obst.state.get_xy_position()

        # only search around obstacle
        (obst_clearance_x_min, obst_clearance_y_min) = self.cart_2d_to_c_idx_or_grid_edge(
                obst_cart_2d[0]-max_robot_to_obst_x_distance, obst_cart_2d[1]-max_robot_to_obst_y_distance)

        (obst_clearance_x_max, obst_clearance_y_max) = self.cart_2d_to_c_idx_or_grid_edge(
                obst_cart_2d[0]+max_robot_to_obst_x_distance, obst_cart_2d[1]+max_robot_to_obst_y_distance)

        for x_idx in range(obst_clearance_x_min, obst_clearance_x_max+1):
            for y_idx in range(obst_clearance_y_min, obst_clearance_y_max+1):

                if point_in_rectangle(self.c_idx_to_cart_2d(x_idx, y_idx), obst_a, obst_b, obst_c):
                    self.grid_map[x_idx, y_idx] = val
                    continue

                r_cart_2d = np.array(self.c_idx_to_cart_2d(x_idx, y_idx))
                
                # check if the edges of the robot overlap with the obstacle
                if minimal_distance_point_to_line(r_cart_2d, obst_a, obst_b) <= self.robot_radius:
                    self.grid_map[x_idx, y_idx] = val
                    continue

                elif minimal_distance_point_to_line(r_cart_2d, obst_b, obst_c) <= self.robot_radius:
                    self.grid_map[x_idx, y_idx] = val
                    continue

                elif minimal_distance_point_to_line(r_cart_2d, obst_c, obst_d) <= self.robot_radius:
                    self.grid_map[x_idx, y_idx] = val
                    continue

                elif minimal_distance_point_to_line(r_cart_2d, obst_d, obst_a) <= self.robot_radius:
                    self.grid_map[x_idx, y_idx] = val 
                    continue

    def occupancy(self, cart_2d: np.ndarray) -> int:
        """ returns the occupancy of the grid cell """
        assert cart_2d.shape == (2,), f"cart_2d not of shape (2,) but {cart_2d.shape}"
        idx = self.cart_2d_to_c_idx(cart_2d[0], cart_2d[1]) 
        return self.c_idx_to_occupancy(*idx)

    def c_idx_to_occupancy(self, x_idx: int, y_idx: int):
        if (x_idx > self.grid_map.shape[0] or
            x_idx < 0):
            raise ValueError(f"x_idx should be in range [0, {self.grid_map.shape[0]}] and is {x_idx}")

        if (y_idx > self.grid_map.shape[1] or
            y_idx < 0):
            raise ValueError(f"y_idx should be in range [0, {self.grid_map.shape[1]}] and is {y_idx}")

        return self.grid_map[x_idx, y_idx]
    
    def shortest_path(self, cart_2d_start: np.ndarray, cart_2d_target: np.ndarray) -> list:
        """ Dijkstra shortest path algorithm. """

        if self.occupancy(cart_2d_start) != 1:
            warnings.warn("the start position is in obstacle space")

        if self.occupancy(cart_2d_target) != 1:
            warnings.warn("the target position is in obstacle space")

        # convert position to indices on the grid
        c_idx_start =  self.cart_2d_to_c_idx(cart_2d_start[0], cart_2d_start[1])
        c_idx_target = self.cart_2d_to_c_idx(cart_2d_target[0], cart_2d_target[1])

        # a visited flag (0 for unvisited, 1 for in the queue, 2 for visited)
        visited = np.zeros(self.grid_map.shape).astype(int)
        previous_cell = np.zeros((self.grid_map.shape[0], self.grid_map.shape[1], 2)).astype(int)
        
        # set all cost to maximal size, except for the starting cell position
        cost = sys.maxsize*np.ones(self.grid_map.shape)
        cost[c_idx_start] = 0
        
        queue = []
        queue.append(c_idx_start)
    
        (x_max, y_max) = self.grid_map.shape
        
        while len(queue) != 0:
            c_idx_temp = queue.pop(0)
            
            # set c_idx_temp to visited
            visited[c_idx_temp] = 2

            x_low = max(c_idx_temp[0]-1, 0)
            x_high = min(c_idx_temp[0]+1, x_max-1)
            y_low = max(c_idx_temp[1]-1, 0)
            y_high = min(c_idx_temp[1]+1, y_max-1)

            # loop though neighboring indexes
            for x_idx in range(x_low, x_high+1):
                for y_idx in range(y_low, y_high+1):

                        c_idx = (x_idx, y_idx)

                        # only compare unvisited cells
                        if visited[c_idx] != 2:

                            # path cannot go through obstacles
                            if self.c_idx_to_occupancy(*c_idx) != 1:

                                # put cell in the queue if not already in there
                                if visited[c_idx] == 0:
                                    visited[c_idx] = 1 
                                    queue.append(c_idx)
                               
                                # update cost and previous cell if lower cost is found 
                                temp_cost = cost[c_idx_temp] + np.linalg.norm(c_idx_temp-np.array(c_idx))
                                if temp_cost < cost[c_idx]:

                                    cost[c_idx] = temp_cost
                                    previous_cell[c_idx[0], c_idx[1], :] = c_idx_temp

        shortest_path_reversed = []
        c_idx_temp = c_idx_target
       
        # find shortest path from target to start
        while not all(x == y for x, y in zip(c_idx_temp, c_idx_start)):

            c_idx_temp = previous_cell[c_idx_temp[0], c_idx_temp[1], :]
            shortest_path_reversed.append(c_idx_temp)

        # reverse list and convert to 2D cartesian position
        if len(shortest_path_reversed) != 0:
            shortest_path_reversed.pop()

        shortest_path = [tuple(cart_2d_start)]

        while len(shortest_path_reversed) != 0:
            shortest_path.append(self.c_idx_to_cart_2d(*shortest_path_reversed.pop()))

        shortest_path.append(tuple(cart_2d_target))

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
            x=[self.robot_cart_2d[1]],
            y=[self.robot_cart_2d[0]],
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
                x0=self.robot_cart_2d[1]-self.robot_radius,
                y0=self.robot_cart_2d[0]-self.robot_radius,
                x1=self.robot_cart_2d[1]+self.robot_radius,
                y1=self.robot_cart_2d[0]+self.robot_radius,
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
        
        # add the obstacles over the gridmap
        for obst in self.obstacles.values():

            # add the name of the obstect
            fig.add_trace(go.Scatter(
                x=[obst.state.pos[1]],
                y=[obst.state.pos[0]],
                text=[obst.name],
                mode="text",
                textfont=dict(
                    color="black",
                    size=18,
                    family="Arail",
                    )
                )
            )

            match obst.properties.type():
                case "sphere" | "cylinder":
                    fig.add_shape(type="circle",
                            xref="x", yref="y",
                            x0=obst.state.pos[1]-obst.properties.radius(),
                            y0=obst.state.pos[0]-obst.properties.radius(),
                            x1=obst.state.pos[1]+obst.properties.radius(),
                            y1=obst.state.pos[0]+obst.properties.radius(),
                            line_color="black",
                            )
                case "box":
                    cos_ol = math.cos(obst.state.ang_p[2])*obst.properties.width()/2
                    sin_ol = math.sin(obst.state.ang_p[2])*obst.properties.width()/2
                    cos_ow = math.cos(obst.state.ang_p[2])*obst.properties.length()/2
                    sin_ow = math.sin(obst.state.ang_p[2])*obst.properties.length()/2

                    fig.add_trace(go.Scatter(y=[obst.state.pos[0]-sin_ol+cos_ow,
                        obst.state.pos[0]-sin_ol-cos_ow,
                        obst.state.pos[0]+sin_ol-cos_ow,
                        obst.state.pos[0]+sin_ol+cos_ow,
                        obst.state.pos[0]-sin_ol+cos_ow],
                        x=[obst.state.pos[1]+cos_ol+sin_ow,
                            obst.state.pos[1]+cos_ol-sin_ow,
                            obst.state.pos[1]-cos_ol-sin_ow,
                            obst.state.pos[1]-cos_ol+sin_ow,
                            obst.state.pos[1]+cos_ol+sin_ow],
                        line_color="black",
                        mode='lines'))

                case "urdf":
                    warnings.warn("the urdf obststacle is not yet implemented")

                case _:
                    raise TypeError(f"Could not recognise obstacle type: {obst.properties.type()}")

        fig.update_layout(
                height=900,
                showlegend= False,
                paper_bgcolor=FIG_BG_COLOR,
                plot_bgcolor=FIG_BG_COLOR,
            )

        fig.update_yaxes(autorange="reversed")
        if save:
            with open(PROJECT_PATH+"dashboard/data/configuration_grid.pickle", "wb") as file:
                pickle.dump(fig, file)
        else:
            fig.show()

    @property
    def grid_map(self):
        return self._grid_map

    @property
    def robot_radius(self):
        return self._robot_radius
