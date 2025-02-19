import sys
from typing import Tuple
import pickle
import math
import warnings
import plotly.graph_objects as go
import numpy as np

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from helper_functions.geometrics import (
        minimal_distance_point_to_line,
        point_in_rectangle,
        do_intersect,
        to_interval_zero_to_two_pi,
        )

from robot_brain.local_planning.graph_based.path_estimator import PathEstimator
from robot_brain.object import Object, FREE, UNKNOWN, MOVABLE, UNMOVABLE
from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH
from robot_brain.state import State
from robot_brain.exceptions import NoPathExistsException

class RectangleObjectPathEstimator(PathEstimator):
    """ The configuration grid map for rectangular objects. """

    def __init__(self,
        cell_size: float,
        grid_x_length: float,
        grid_y_length: float,
        objects: dict,
        obst_cart_2d: np.ndarray,
        obst_name: str,
        obst_x_length: float,
        obst_y_length: float,
        n_orientations: int,
        single_orientation = False,
        orientation = 0.0):

        PathEstimator.__init__(self, cell_size, grid_x_length, grid_y_length,
                objects, obst_cart_2d, obst_name, n_orientations, single_orientation, orientation)
        self._obst_x_length = obst_x_length
        self._obst_y_length = obst_y_length

        # TODO: make this work with 2 dimensional values (and then grid_map is also 2D), and with 3 dimensional grid map.
        self._grid_map = np.zeros((
            int(self.grid_x_length/self.cell_size),
            int(self.grid_y_length/self.cell_size),
            int(n_orientations)))

        self.setup()

    def _setup_circle_object(self, obst: Object, val: int, r_orien: float, r_orien_idx: int):
        """ For a circular object set all overlapping with grid
        cells (representing the object) to a integer value. """

        # cos_rl = cos(orientation_of_obst) * obst_length_in_x / 2
        cos_rl = math.cos(r_orien)*self.obst_y_length/2
        sin_rl = math.sin(r_orien)*self.obst_y_length/2
        cos_rw = math.cos(r_orien)*self.obst_x_length/2
        sin_rw = math.sin(r_orien)*self.obst_x_length/2

        obst_xy = obst.state.get_xy_position()
        max_obst_obst_distance= (math.sqrt(self.obst_x_length**2 + self.obst_y_length**2))/2 + obst.properties.radius()

        # only search around object
        (obst_clearance_x_min, obst_clearance_y_min) = self._cart_2d_to_c_idx_or_grid_edge(
                obst_xy[0]-max_obst_obst_distance, obst_xy[1]-max_obst_obst_distance)
        (obst_clearance_x_max, obst_clearance_y_max) = self._cart_2d_to_c_idx_or_grid_edge(
                obst_xy[0]+max_obst_obst_distance, obst_xy[1]+max_obst_obst_distance)

        for x_idx in range(obst_clearance_x_min, obst_clearance_x_max+1):
            for y_idx in range(obst_clearance_y_min, obst_clearance_y_max+1):
                #  closeby  cells are always in collision with the object
                if (np.linalg.norm(self._c_idx_to_cart_2d(x_idx, y_idx)-obst_xy) <= \
                obst.properties.radius() + min(self.obst_x_length, self.obst_y_length) / 2):
                    self.grid_map[x_idx, y_idx, r_orien_idx] = val
                    continue

               # corner points of the object
                a = self._c_idx_to_cart_2d(x_idx, y_idx) \
                        + np.array([-sin_rl+cos_rw, cos_rl+sin_rw])

                b =  self._c_idx_to_cart_2d(x_idx, y_idx) \
                        + np.array([-sin_rl-cos_rw, cos_rl-sin_rw])

                c =  self._c_idx_to_cart_2d(x_idx, y_idx) \
                        + np.array([+sin_rl-cos_rw, -cos_rl-sin_rw])

                d =  self._c_idx_to_cart_2d(x_idx, y_idx) \
                        + np.array([sin_rl+cos_rw, -cos_rl+sin_rw])

                obst_r = obst.properties.radius()

                # check if the edges of the obst overlap with the object
                if minimal_distance_point_to_line(obst_xy, a, b) <= obst_r:
                    self.grid_map[x_idx, y_idx, r_orien_idx] = val
                    continue

                elif minimal_distance_point_to_line(obst_xy, b, c) <= obst_r:
                    self.grid_map[x_idx, y_idx, r_orien_idx] = val
                    continue

                elif minimal_distance_point_to_line(obst_xy, c, d) <= obst_r:
                    self.grid_map[x_idx, y_idx, r_orien_idx] = val
                    continue

                elif minimal_distance_point_to_line(obst_xy, d, a) <= obst_r:
                    self.grid_map[x_idx, y_idx, r_orien_idx] = val
                    continue

    def _setup_rectangular_object(self, obst: Object, val: int, r_orien: float, r_orien_idx: int):
        """ For a rectangular object set all overlapping with grid cells
        (representing the object) to a integer value. """

        # object space or some other space, Gijs Groote 4 oct 2022.
        cos_rl = math.cos(r_orien)*self.obst_y_length/2
        sin_rl = math.sin(r_orien)*self.obst_y_length/2
        cos_rw = math.cos(r_orien)*self.obst_x_length/2
        sin_rw = math.sin(r_orien)*self.obst_x_length/2

        cos_ol = math.cos(obst.state.ang_p[2])*obst.properties.width()/2
        sin_ol = math.sin(obst.state.ang_p[2])*obst.properties.width()/2
        cos_ow = math.cos(obst.state.ang_p[2])*obst.properties.length()/2
        sin_ow = math.sin(obst.state.ang_p[2])*obst.properties.length()/2
        # corner points of the object
        obst_a = np.array([obst.state.pos[0]-sin_ol+cos_ow, obst.state.pos[1]+cos_ol+sin_ow])
        obst_b = np.array([obst.state.pos[0]-sin_ol-cos_ow, obst.state.pos[1]+cos_ol-sin_ow])
        obst_c = np.array([obst.state.pos[0]+sin_ol-cos_ow, obst.state.pos[1]-cos_ol-sin_ow])
        obst_d = np.array([obst.state.pos[0]+sin_ol+cos_ow, obst.state.pos[1]-cos_ol+sin_ow])

        max_obst_to_obst_x_distance = abs(sin_rl) + abs(cos_rw) + abs(sin_ol) + abs(cos_ow)
        max_obst_to_obst_y_distance = abs(cos_rl) + abs(sin_rw) + abs(cos_ol) + abs(sin_ow)
        obst_cart_2d = obst.state.get_xy_position()

        # only search around object
        (x_min, y_min) = self._cart_2d_to_c_idx_or_grid_edge(obst_cart_2d[0]\
                -max_obst_to_obst_x_distance, obst_cart_2d[1]-max_obst_to_obst_y_distance)
        (x_max, y_max) = self._cart_2d_to_c_idx_or_grid_edge(obst_cart_2d[0]\
                +max_obst_to_obst_x_distance, obst_cart_2d[1]+max_obst_to_obst_y_distance)

        # orientation with cos/sin could make x_min > x_max
        obst_clearance_x_min = min(x_min, x_max)
        obst_clearance_x_max = max(x_min, x_max)
        obst_clearance_y_min = min(y_min, y_max)
        obst_clearance_y_max = max(y_min, y_max)

        for x_idx in range(obst_clearance_x_min, obst_clearance_x_max+1):
            for y_idx in range(obst_clearance_y_min, obst_clearance_y_max+1):

                # if the center of the obst is in the object, then they are in collision
                # NOTE: this assumes the 3 points to form a rectangle
                # if the obstect is rotated it could form a diamond shape instead of a rectangle
                if point_in_rectangle(np.array(self._c_idx_to_cart_2d(x_idx, y_idx)),
                        obst_a, obst_b, obst_c):
                    self.grid_map[x_idx, y_idx, r_orien_idx] = val
                    continue

                # corner points of the object
                a = self._c_idx_to_cart_2d(x_idx, y_idx) \
                        + np.array([-sin_rl+cos_rw, cos_rl+sin_rw])

                b =  self._c_idx_to_cart_2d(x_idx, y_idx) \
                        + np.array([-sin_rl-cos_rw, cos_rl-sin_rw])

                c =  self._c_idx_to_cart_2d(x_idx, y_idx) \
                        + np.array([+sin_rl-cos_rw, -cos_rl-sin_rw])

                d =  self._c_idx_to_cart_2d(x_idx, y_idx) \
                        + np.array([sin_rl+cos_rw, -cos_rl+sin_rw])

                # check if the edges of the object overlap with the object
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

                    self.grid_map[x_idx, y_idx, r_orien_idx] = val

    def _p_idx_to_occupancy(self, x_idx: int, y_idx: int, orien_idx: int) -> int:
        """ returns the occupancy of a grid cell """
        if not all([isinstance(x_idx, int), isinstance(y_idx, int), isinstance(orien_idx, int)]):
            raise TypeError("all arguements should be intergers")

        if (x_idx >= self.grid_map.shape[0] or
            x_idx < 0):
            raise ValueError(f"x_idx should be in [0, {self.grid_map.shape[0]}] and is {x_idx}")

        if (y_idx >= self.grid_map.shape[1] or
            y_idx < 0):
            raise ValueError(f"y_idx should be in [0, {self.grid_map.shape[1]}] and is {y_idx}")

        if (orien_idx >= self.grid_map.shape[2] or
            orien_idx < 0):
            raise ValueError(f"orien_idx should be in [0, {self.grid_map.shape[2]}] and is {orien_idx}")

        return self.grid_map[x_idx, y_idx, orien_idx]

    def occupancy(self, pose_2d: np.ndarray) -> int:
        """ return the occupancy of a grid cell from a 2d pose. """
        if isinstance(pose_2d, list):
            pose_2d = np.array(pose_2d)

        if pose_2d.shape == ((2,), (2,1)):
            pose_2d = np.append(pose_2d, [0])

        idx = self._pose_2d_to_p_idx(pose_2d)
        return self._p_idx_to_occupancy(*idx)

    def search_path(self, pose_2d_start: np.ndarray, pose_2d_target:np.ndarray) -> list:
        """ use the Dijkstra algorithm to find the shortest path. """

        if isinstance(pose_2d_start, State):
            pose_2d_start = pose_2d_start.get_2d_pose()

        if isinstance(pose_2d_target, State):
            pose_2d_target = pose_2d_target.get_2d_pose()

        # convert angles to inteval [0, 2*pi)
        pose_2d_start[2] = to_interval_zero_to_two_pi(pose_2d_start[2])
        pose_2d_target[2] = to_interval_zero_to_two_pi(pose_2d_target[2])

        if self.occupancy(pose_2d_start) == UNMOVABLE:
            raise NoPathExistsException("Target state in object space")

        if self.occupancy(pose_2d_start) != FREE:
            # convert start position to closeby position
            # TODO: add a small dist orientation wise
            for small_dist in [[0.2, 0.0], [-0.2, 0.0],
                    [0.0, 0.2], [0.0, -0.2],
                    [0.15, 0.15], [-0.15, 0.15],
                    [0.15, -0.15], [-0.15, -0.15],
                    [0.5, 0.0], [-0.5, 0.0],
                    [0.0, 0.5], [0.0, -0.5],
                    [0.5, 0.5], [-0.5, 0.5],
                    [0.5, -0.5], [-0.5, -0.5]]:

                closeby_start_x = pose_2d_start[0] + small_dist[0]
                closeby_start_y = pose_2d_start[1] + small_dist[1]

                if self.occupancy([closeby_start_x, closeby_start_y, 0.0]) == FREE:
                    pose_2d_start = np.array([closeby_start_x, closeby_start_y, 0.0])
                    break
            warnings.warn("the start position is in movable or unkown space")

        if self.occupancy(pose_2d_target) != FREE:
            warnings.warn("the target position is in movable or unknown space")

        # convert position to indices on the grid
        p_idx_start = self._pose_2d_to_p_idx(pose_2d_start)
        p_idx_target = self._pose_2d_to_p_idx(pose_2d_target)


        # first try to plan only in free space, then also in movable and unknown space
        for allowed_subspaces in [[FREE], [FREE, MOVABLE, UNKNOWN]]:

            # a visited flag (0 for unvisited, 1 for in the queue, 2 for visited)
            visited = np.zeros((self.grid_map.shape[0],
                self.grid_map.shape[1], self.grid_map.shape[2])).astype(int)
            previous_cell = np.zeros((self.grid_map.shape[0],
                self.grid_map.shape[1], self.grid_map.shape[2], 3)).astype(int)

            # set all cost to maximal size, except for the starting cell position
            cost = sys.maxsize*np.ones(self.grid_map.shape)
            cost[p_idx_start] = 0

            queue = []
            queue.append(p_idx_start)

            (x_max, y_max, orien_max) = self.grid_map.shape

            while len(queue) != 0:
                cell_pose_temp = queue.pop(0)

                # set cell_pose_temp to visited
                visited[cell_pose_temp[0], cell_pose_temp[1], cell_pose_temp[2]] = 2

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

                            p_idx = (x_idx, y_idx, orien_idx)

                            # only compare unvisited cells
                            if visited[p_idx] != 2:

                                # path cannot go through objects
                                if self._p_idx_to_occupancy(*p_idx) in allowed_subspaces:

                                    # put cell in the queue if not already in there
                                    if visited[p_idx] == 0:
                                        visited[p_idx] = 1
                                        queue.append(p_idx)

                                    # update cost and previous cell is a lower cost to a cell is found
                                    temp_cost = cost[cell_pose_temp]\
                                    +np.linalg.norm(cell_pose_temp-np.array(p_idx))
                                    if temp_cost < cost[p_idx]:

                                        cost[p_idx] = temp_cost
                                        previous_cell[x_idx, y_idx, orien_idx, :] = cell_pose_temp

            if cost[p_idx_target] < sys.maxsize:
                break

        if cost[p_idx_target] == sys.maxsize:
            raise NoPathExistsException("Dijkstra algorithm is unable to connect start to target state")

        shortest_path_reversed = []
        cell_pose_temp = p_idx_target

        # find path from target to start
        while not all(x == y for x, y in zip(cell_pose_temp, p_idx_start)):
            cell_pose_temp = previous_cell[cell_pose_temp[0],
                    cell_pose_temp[1], cell_pose_temp[2], :]
            shortest_path_reversed.append(cell_pose_temp)

        # remove start position
        if len(shortest_path_reversed) != 0:
            shortest_path_reversed.pop()

        shortest_path = [tuple(pose_2d_start)]

        # reverse list and convert to 2D poses
        while len(shortest_path_reversed) != 0:
            shortest_path.append(self._p_idx_to_pose_2d(*shortest_path_reversed.pop()))

        shortest_path.append(tuple(pose_2d_target))

        self.shortest_path = shortest_path

        return shortest_path

    def _pose_2d_to_p_idx(self, pose_2d: np.ndarray) -> Tuple[int, int, int]:
        """ returns the index of the cell a 2D pose (x_position, y_position, orientation). """
        if pose_2d.shape != (3,):
            if pose_2d.shape == (2,):
                pose_2d = np.array([pose_2d[0], pose_2d[1], 0])
            else:
                raise IndexError(f"the shape of pose_2d is {pose_2d.shape} and should be (3,)")
        if pose_2d[2] >= 2*math.pi or pose_2d[2] < 0:
            raise ValueError(f"orientation: {pose_2d[2]} is not in the interval [0, 2*pi)")

        normalised_orien= pose_2d[2]/(2*math.pi) + 1/(self.n_orientations*2)
        if normalised_orien >= 1:
            normalised_orien = 0

        orien_idx = int(normalised_orien*self.n_orientations)

        (x_idx, y_idx) = self._cart_2d_to_c_idx(pose_2d[0], pose_2d[1])

        return (x_idx, y_idx, orien_idx)

    def _p_idx_to_pose_2d(self, x_idx: int, y_idx: int, orien_idx: int) -> Tuple[float, float, float]:
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

    def update(self):
        """ refresh grid map. """
        self.grid_map = np.zeros((
            int(self.grid_x_length/self.cell_size),
            int(self.grid_y_length/self.cell_size),
            int(self.n_orientations)))
        self.setup()

    def visualise(self, orien_idx:int=0, save:bool=True):
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
        extended_grid_map[1:-1,1:-1] = np.reshape(self.grid_map[:,:,orien_idx],
                (int(self.grid_x_length/self.cell_size), int(self.grid_y_length/self.cell_size)))

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

        # put the object on the map
        r_orien = orien_idx/self.n_orientations*2*math.pi

        cos_rl = math.cos(r_orien)*self.obst_y_length/2
        sin_rl = math.sin(r_orien)*self.obst_y_length/2
        cos_rw = math.cos(r_orien)*self.obst_x_length/2
        sin_rw = math.sin(r_orien)*self.obst_x_length/2

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

        fig.add_trace(go.Scatter(
            y=[self.obst_cart_2d[0]-sin_rl+cos_rw,
            self.obst_cart_2d[0]-sin_rl-cos_rw,
            self.obst_cart_2d[0]+sin_rl-cos_rw,
            self.obst_cart_2d[0]+sin_rl+cos_rw,
            self.obst_cart_2d[0]-sin_rl+cos_rw],
            x=[self.obst_cart_2d[1]+cos_rl+sin_rw,
                self.obst_cart_2d[1]+cos_rl-sin_rw,
                self.obst_cart_2d[1]-cos_rl-sin_rw,
                self.obst_cart_2d[1]-cos_rl+sin_rw,
                self.obst_cart_2d[1]+cos_rl+sin_rw],
            line_color="black",
            mode='lines'
            )
        )

        # add the boundaries of the map 
        fig.add_shape(type="rect",
            x0=self.grid_y_length/2, y0=self.grid_x_length/2,
            x1=-self.grid_y_length/2, y1=-self.grid_x_length/2,
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
            with open(PROJECT_PATH+"dashboard/data/configuration_grid.pickle", "wb") as file:
                pickle.dump(fig, file)
        else:
            fig.show()

    @property
    def obst_x_length(self):
        return self._obst_x_length

    @property
    def obst_y_length(self):
        return self._obst_y_length
