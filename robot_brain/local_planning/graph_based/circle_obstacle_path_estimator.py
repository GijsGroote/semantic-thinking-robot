import sys
import math
import warnings
import pickle
from typing import Tuple
import numpy as np
import plotly.graph_objects as go

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from helper_functions.geometrics import minimal_distance_point_to_line, point_in_rectangle
from robot_brain.local_planning.graph_based.path_estimator import PathEstimator
from robot_brain.object import Object, FREE, MOVABLE, UNKNOWN, UNMOVABLE
from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH
from robot_brain.state import State
from robot_brain.exceptions import NoPathExistsException

class CircleObstaclePathEstimator(PathEstimator):
    """ The configuration grid map for circular objects. """

    def __init__(self,
            cell_size: float,
            grid_x_length: float,
            grid_y_length: float,
            objects: dict,
            obst_cart_2d: np.ndarray,
            obst_name: str,
            obst_radius: float):

        PathEstimator.__init__(self, cell_size, grid_x_length, grid_y_length,
                objects, obst_cart_2d, obst_name, 1, True, 0.0)

        self._obst_radius = obst_radius
        self._grid_map = np.zeros((
            int(self.grid_x_length/self.cell_size),
            int(self.grid_y_length/self.cell_size)))

        self.setup()

    def _setup_circle_object(self, obst: Object, val: int, r_orien: float, r_orien_idx: int):
        """ For a circular object set all overlapping with grid cells
        (representing the object) to a integer value. """

        obst_cart_2d = obst.state.get_xy_position()

        # only search around object
        (obst_clearance_x_min, obst_clearance_y_min) = self._cart_2d_to_c_idx_or_grid_edge(
                obst_cart_2d[0]-(self.obst_radius + obst.properties.radius()),
                obst_cart_2d[1]-(self.obst_radius + obst.properties.radius()))

        (obst_clearance_x_max, obst_clearance_y_max) = self._cart_2d_to_c_idx_or_grid_edge(
                obst_cart_2d[0]+(self.obst_radius + obst.properties.radius()),
                obst_cart_2d[1]+(self.obst_radius + obst.properties.radius()))

        for x_idx in range(obst_clearance_x_min, obst_clearance_x_max+1):
            for y_idx in range(obst_clearance_y_min, obst_clearance_y_max+1):
                #  closeby cells are always in collision with the object
                if np.linalg.norm(self._c_idx_to_cart_2d(x_idx, y_idx)-obst_cart_2d) <= obst.properties.radius() + self.obst_radius:
                    self.grid_map[x_idx, y_idx] = val

    def _setup_rectangular_object(self, obst: Object, val: int, r_orien: float, r_orien_idx: int):
        """ For a rectangular object set all overlapping with grid cells
        (representing the object) to a integer value. """

        cos_ol = math.cos(obst.state.ang_p[2])*obst.properties.width()/2
        sin_ol = math.sin(obst.state.ang_p[2])*obst.properties.width()/2
        cos_ow = math.cos(obst.state.ang_p[2])*obst.properties.length()/2
        sin_ow = math.sin(obst.state.ang_p[2])*obst.properties.length()/2

        # corner points of the object
        obst_a = np.array([obst.state.pos[0]-sin_ol+cos_ow, obst.state.pos[1]+cos_ol+sin_ow])
        obst_b = np.array([obst.state.pos[0]-sin_ol-cos_ow, obst.state.pos[1]+cos_ol-sin_ow])
        obst_c = np.array([obst.state.pos[0]+sin_ol-cos_ow, obst.state.pos[1]-cos_ol-sin_ow])
        obst_d = np.array([obst.state.pos[0]+sin_ol+cos_ow, obst.state.pos[1]-cos_ol+sin_ow])

        max_obst_to_obst_x_distance = self.obst_radius + abs(sin_ol) + abs(cos_ow)
        max_obst_to_obst_y_distance = self.obst_radius + abs(cos_ol) + abs(sin_ow)

        obst_cart_2d = obst.state.get_xy_position()

        # only search around object
        (obst_clearance_x_min, obst_clearance_y_min) = self._cart_2d_to_c_idx_or_grid_edge(
                obst_cart_2d[0]-max_obst_to_obst_x_distance,
                obst_cart_2d[1]-max_obst_to_obst_y_distance)

        (obst_clearance_x_max, obst_clearance_y_max) = self._cart_2d_to_c_idx_or_grid_edge(
                obst_cart_2d[0]+max_obst_to_obst_x_distance,
                obst_cart_2d[1]+max_obst_to_obst_y_distance)

        for x_idx in range(obst_clearance_x_min, obst_clearance_x_max+1):
            for y_idx in range(obst_clearance_y_min, obst_clearance_y_max+1):

                if point_in_rectangle(self._c_idx_to_cart_2d(x_idx, y_idx), obst_a, obst_b, obst_c):
                    self.grid_map[x_idx, y_idx] = val
                    continue

                r_cart_2d = np.array(self._c_idx_to_cart_2d(x_idx, y_idx))

                # check if the edges of the obst overlap with the obj
                if minimal_distance_point_to_line(r_cart_2d, obst_a, obst_b) <= self.obst_radius:
                    self.grid_map[x_idx, y_idx] = val
                    continue

                elif minimal_distance_point_to_line(r_cart_2d, obst_b, obst_c) <= self.obst_radius:
                    self.grid_map[x_idx, y_idx] = val
                    continue

                elif minimal_distance_point_to_line(r_cart_2d, obst_c, obst_d) <= self.obst_radius:
                    self.grid_map[x_idx, y_idx] = val
                    continue

                elif minimal_distance_point_to_line(r_cart_2d, obst_d, obst_a) <= self.obst_radius:
                    self.grid_map[x_idx, y_idx] = val
                    continue

    def occupancy(self, cart_2d: np.ndarray) -> int:
        """ returns the occupancy of the grid cell """

        if isinstance(cart_2d, list):
            cart_2d = np.array(cart_2d)

        if cart_2d.shape == (3,):
            cart_2d = cart_2d[0:2]

        assert cart_2d.shape == (2,), f"cart_2d not of shape (2,) but {cart_2d.shape}"
        idx = self._cart_2d_to_c_idx(cart_2d[0], cart_2d[1])
        return self._c_idx_to_occupancy(*idx)

    def _c_idx_to_occupancy(self, x_idx: int, y_idx: int):
        if (x_idx > self.grid_map.shape[0] or
            x_idx < 0):
            raise ValueError(f"x_idx should be in [0, {self.grid_map.shape[0]}] and is {x_idx}")

        if (y_idx > self.grid_map.shape[1] or
            y_idx < 0):
            raise ValueError(f"y_idx should be in [0, {self.grid_map.shape[1]}] and is {y_idx}")

        return self.grid_map[x_idx, y_idx]

    def search_path(self, cart_2d_start: np.ndarray, cart_2d_target: np.ndarray) -> list:
        """ Dijkstra shortest path algorithm. """

        if isinstance(cart_2d_start, State):
            cart_2d_start = cart_2d_start.get_xy_position()

        if isinstance(cart_2d_target, State):
            cart_2d_target= cart_2d_target.get_xy_position()

        if self.occupancy(cart_2d_target) == UNMOVABLE:
            raise NoPathExistsException("Target state in object space")

        if self.occupancy(cart_2d_start) != FREE:
            warnings.warn(f"the start position {cart_2d_start} is in movable or unknown space")

        if self.occupancy(cart_2d_target) != FREE:
            warnings.warn(f"the target position {cart_2d_target} is in movable or unknown space")

        # convert position to indices on the grid
        c_idx_start =  self._cart_2d_to_c_idx(cart_2d_start[0], cart_2d_start[1])
        c_idx_target = self._cart_2d_to_c_idx(cart_2d_target[0], cart_2d_target[1])

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

                        # path cannot go through objects
                        if self._c_idx_to_occupancy(*c_idx) != UNMOVABLE:

                            # put cell in the queue if not already in there
                            if visited[c_idx] == 0:
                                visited[c_idx] = 1
                                queue.append(c_idx)

                            # update cost and previous cell if lower cost is found
                            temp_cost = cost[c_idx_temp] + np.linalg.norm(c_idx_temp-np.array(c_idx))
                            if temp_cost < cost[c_idx]:

                                cost[c_idx] = temp_cost
                                previous_cell[c_idx[0], c_idx[1], :] = c_idx_temp

        # no shortest path was found
        if cost[c_idx_target] == sys.maxsize:
            raise NoPathExistsException("Dijkstra algorithm could not find a path from start to target state.")

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
            shortest_path.append(self._c_idx_to_cart_2d(*shortest_path_reversed.pop()))

        shortest_path.append(tuple(cart_2d_target))

        self.shortest_path = shortest_path

        return shortest_path

    def update(self):
        self.grid_map = np.zeros((
            int(self.grid_x_length/self.cell_size),
            int(self.grid_y_length/self.cell_size)))
        self.setup()

    def visualise(self, save: bool=True):
        """ Display the configuration grid map. """

        dcolorsc = [[0, '#90ee90'], [0.25, '#90ee90'],
                [0.25, '#19d3f3'],[0.5, '#19d3f3'],
                [0.5, '#e763fa'], [0.75, '#e763fa'],
                [0.75, '#ab63fa'], [1.0, '#ab63fa']]

        tickvals = [3/8, 9/8, 15/8, 21/8]
        ticktext = ["free", "movable", "unknown", "obstacle"]

        extended_grid_map = np.zeros((int(self.grid_x_length/self.cell_size+2.0),
            int(self.grid_y_length/self.cell_size+2.0)))
        extended_grid_map[:,0] = 0
        extended_grid_map[:,-1] = 1
        extended_grid_map[0,:] = 2
        extended_grid_map[-1,:] = 3
        extended_grid_map[1:-1,1:-1] = self.grid_map

        trace = go.Heatmap(
               x=list(np.arange((-(self.grid_y_length))/2,
                   (2*self.cell_size+(self.grid_y_length))/2, self.cell_size)),
                y=list(np.arange((-(self.grid_x_length))/2,
                    (2*self.cell_size+(self.grid_x_length))/2, self.cell_size)),
                z=extended_grid_map,
                type = "heatmap",
                colorscale = dcolorsc,
                colorbar = dict(thickness=25,
                                tickvals=tickvals,
                                ticktext=ticktext))

        fig = go.Figure(data=trace)

        fig.add_trace(go.Scatter(
            x=[self.obst_cart_2d[1]],
            y=[self.obst_cart_2d[0]],
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
                x0=self.obst_cart_2d[1]-self.obst_radius,
                y0=self.obst_cart_2d[0]-self.obst_radius,
                x1=self.obst_cart_2d[1]+self.obst_radius,
                y1=self.obst_cart_2d[0]+self.obst_radius,
                line_color="black",
        )

        # add the boundaries of the map
        fig.add_shape(type="rect",
                x0=(self.grid_y_length+self.cell_size)/2,
                y0=(self.grid_x_length+self.cell_size)/2,
                x1=(-self.grid_y_length+self.cell_size)/2,
                y1=(-self.grid_x_length+self.cell_size)/2,
                line_color="black",
                )

        # add the objects over the gridmap
        for obst in self.objects.values():

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

            if isinstance(obst.properties, CylinderObstacle):
                fig.add_shape(type="circle",
                        xref="x", yref="y",
                        x0=obst.state.pos[1]-obst.properties.radius(),
                        y0=obst.state.pos[0]-obst.properties.radius(),
                        x1=obst.state.pos[1]+obst.properties.radius(),
                        y1=obst.state.pos[0]+obst.properties.radius(),
                        line_color="black",
                        )

            elif isinstance(obst.properties, BoxObstacle):
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

            else:
                raise TypeError(f"Could not recognise object type: {obst.properties.type()}")

        fig.update_layout(
                height=900,
                showlegend= False,
                paper_bgcolor=FIG_BG_COLOR,
                plot_bgcolor=FIG_BG_COLOR,
            )

        fig.update_yaxes(autorange="reversed")
        if save:
            with open(PROJECT_PATH+"dashboard/data/cgrid.pickle", "wb") as file:
                pickle.dump(fig, file)
        else:
            fig.show()

    @property
    def obst_radius(self):
        return self._obst_radius
