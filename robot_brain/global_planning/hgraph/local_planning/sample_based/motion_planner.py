from abc import ABC, abstractmethod
import math
import pickle
import sys
import time
import plotly.graph_objects as go
from sortedcontainers import SortedDict
import numpy as np

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.obstacle import Obstacle
from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH
from robot_brain.state import State
from robot_brain.global_planning.hgraph.local_planning.graph_based.rectangle_obstacle_configuration_grid_map import RectangleObstacleConfigurationGridMap
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_obstacle_configuration_grid_map import CircleObstacleConfigurationGridMap
from robot_brain.global_planning.hgraph.local_planning.graph_based.configuration_grid_map import ConfigurationGridMap
from helper_functions.geometrics import to_interval_zero_to_two_pi


class MotionPlanner(ABC):
    """
    Motion planner that finds a start to target position for pushing and driving tasks.
    """
    def __init__(self, grid_x_length: float,
            grid_y_length: float,
            obstacle: Obstacle,
            obstacles: dict,
            step_size: float,
            search_size: float,
            include_orien=bool,
            configuration_grid_map=None):

        self.grid_x_length = grid_x_length
        self.grid_y_length = grid_y_length
        self.obstacle = obstacle

        assert step_size < search_size,\
                "step size must be smaller than search size,"\
                f"step_size={step_size}, search_size={search_size}"
        self.step_size = step_size
        self.search_size = search_size
        self.start_time_search = 0

        self.source_tree_key = 0
        self.target_tree_key = 1

        self.samples = {}
        self.shortest_paths = SortedDict({})
        self.shortest_path = None
        self.x_sorted = SortedDict({})
        self.y_sorted = SortedDict({})
        self.n_samples = 0
        self.include_orien = include_orien

        if include_orien:
            self.orien_sorted = SortedDict({})

        if isinstance(configuration_grid_map, ConfigurationGridMap):
            if isinstance(obstacle.properties, CylinderObstacle):
                assert isinstance(configuration_grid_map, CircleObstacleConfigurationGridMap),\
                    "obstacle is CylinderObstacle, conf_grid_map should be of "\
                    f"type CircleObstacleConfigurationGridMap and is {type(configuration_grid_map)}"
            elif isinstance(obstacle.properties, BoxObstacle):
                assert isinstance(configuration_grid_map, RectangleObstacleConfigurationGridMap),\
                    "obstacle is BoxObstacle, conf_grid_map should be of type"\
                    f" RectangleObstacleConfigurationGridMap and is {type(configuration_grid_map)}"

            self.configuration_grid_map = configuration_grid_map

        else:
            if isinstance(obstacle.properties, CylinderObstacle):
                self.configuration_grid_map = CircleObstacleConfigurationGridMap(
                    cell_size=0.2,
                    grid_x_length=grid_x_length,
                    grid_y_length=grid_y_length,
                    obstacles=obstacles,
                    obst_cart_2d=obstacle.state.get_xy_position(),
                    obst_name=obstacle.name,
                    obst_radius=obstacle.properties.radius())

            elif isinstance(obstacle.properties, BoxObstacle):
                self.configuration_grid_map = RectangleObstacleConfigurationGridMap(
                    cell_size=0.2,
                    grid_x_length=grid_x_length,
                    grid_y_length=grid_y_length,
                    obstacles=obstacles,
                    obst_cart_2d=obstacle.state.get_xy_position(),
                    obst_name=obstacle.name,
                    n_orientations= 36,
                    obst_x_length=obstacle.properties.length(),
                    obst_y_length=obstacle.properties.width())
            else:
                raise ValueError("Obstacle type is unknown")

    @abstractmethod
    def setup(self, start_sample, target_sample):
        """ initialise the start and target samples. """

    @abstractmethod
    def search_path(self, start: State, target: State) -> tuple:
        """ search for a path between start and target state. """

    @abstractmethod
    def create_random_sample(self):
        """ Randomly generates a sample in free, movable or unknown space. """

    def add_sample(self, sample: list, prev_key: int, cost_to_source) -> int:
        """ adds sample to all existing samples and return unique key generated. """

        # check if the x and y positions already exist
        assert not self.x_sorted.__contains__(sample[0]),\
                f"x position {sample[0]} already exist in sorted x positions"
        assert not self.y_sorted.__contains__(sample[1]),\
                f"y position {sample[1]} already exist in sorted y positions"
        if self.include_orien:
            assert not self.orien_sorted.__contains__(sample[2]),\
                    f"orientation {sample[2]} already exist in sorted orientation"
        if len(sample) == 2:
            sample = [sample[0], sample[1], 0]

        key = self.create_unique_id()

        self.samples[prev_key]["next_sample_keys"].append(key)

        self.samples[key] = {
                "pose": [sample[0], sample[1], sample[2]],
                "cost_to_source": cost_to_source,
                "prev_sample_key": prev_key,
                "next_sample_keys": [],
                "in_tree": self.samples[prev_key]["in_tree"]}

        self.x_sorted[sample[0]] = key
        self.y_sorted[sample[1]]= key
        if self.include_orien:
            self.orien_sorted[sample[2]]= key

        self.n_samples += 1

        return key

    @abstractmethod
    def project_to_connectivity_graph(self, sample: list, project_to_sample_key: int) -> list:
        """ projects the sample closer to the closest existing sample in the connectivity graphs. """

    @abstractmethod
    def check_connecitvity(self, sample1: tuple, sample2: tuple) -> bool:
        """ check if 2 samples can be connected using a local planner. """

    def get_closest_sample_key(self, sample: list) -> int:
        """ Search for closest points in x and y """

        # speed up, only compare closeby samples before comparing to all existing samples
        test_closest_keys = self.get_closeby_sample_keys(sample, 10*self.grid_x_length/self.n_samples)

        if len(test_closest_keys) > 0:
            closest_keys = test_closest_keys
        else:
            test_closest_keys = self.get_closeby_sample_keys(sample, 100*self.grid_x_length/self.n_samples)
            if len(test_closest_keys) > 0:
                closest_keys = test_closest_keys
            else:
                closest_keys = self.samples.keys()

        closest_sample_key = None
        closest_distance = sys.float_info.max
        for key in closest_keys:

            temp_dist = self.distance(sample, self.samples[key]["pose"])
            if temp_dist < closest_distance:
                closest_sample_key = key
                closest_distance = temp_dist

        if closest_sample_key is None:
            raise ValueError("could not found a closest sample, which should be impossible")
        return closest_sample_key

    @abstractmethod
    def distance(self, sample1: list, sample2: list) -> float:
        """ returns distance measurement between 2 samples. """

    def get_closeby_sample_keys(self, sample: list, radius: float) -> set:
        """ return the keys of samples which are less than 2*radius manhattan distance to sample. """
        x_keys = set()
        for x_key in self.x_sorted.irange(sample[0]-radius, sample[0]+radius):
            x_keys.add(self.x_sorted[x_key])

        y_keys = set()
        for y_key in self.y_sorted.irange(sample[1]-radius, sample[1]+radius):
            y_keys.add(self.y_sorted[y_key])

        return x_keys.intersection(y_keys)

    def paths_converged_test(self) -> bool:
        """ test is the shortest path converged. """

        planning_time = time.time() - self.start_time_search

        if planning_time > 0.05:
            raise StopIteration("It takes to long to find a path, halt.")

        if len(self.shortest_paths) < 5:
            return False

        elif self.shortest_paths.peekitem(0)[0] * 1.10 > self.shortest_paths.peekitem(4)[0]:
            # return True when the 5th shortest path is at most 10% longer than the shortest path
            return True
        else:
            return False

    def create_unique_id(self) -> int:
        """ creates and returns a unique id. """
        unique_id = len(self.samples)

        while unique_id in self.samples.keys():
            unique_id += 1

        return unique_id

    def visualise(self, save=True):
        """ Visualise the connectivity graph. """

        fig = go.Figure()

        source_style = dict(showlegend = True,
                name="Source Tree", legendgroup=1, line=dict(color="blue"))
        target_style = dict(showlegend = True,
                name="Target Tree", legendgroup=2, line=dict(color="green"))
        connect_style = dict(showlegend = True,
                name="Connect Trees", legendgroup=3, line=dict(color="orange"))

        # loop through samples
        for sample in self.samples.values():

            prev_sample = self.samples[sample["prev_sample_key"]]["pose"]

            if sample["in_tree"] == self.source_tree_key:
                fig.add_scatter(y=[prev_sample[0], sample["pose"][0]],
                        x=[prev_sample[1], sample["pose"][1]], **source_style)
                source_style["showlegend"] = False

            elif sample["in_tree"] == self.target_tree_key:
                fig.add_scatter(y=[prev_sample[0], sample["pose"][0]], x=[prev_sample[1], sample["pose"][1]], **target_style)
                target_style["showlegend"] = False
            else:
                raise ValueError(f"in_tree in unknown and is {sample['in_tree']}")

        for value in self.shortest_paths.values():
            sample1 = self.samples[value["sample1_key"]]
            sample2 = self.samples[value["sample2_key"]]
            fig.add_scatter(y=[sample1["pose"][0], sample2["pose"][0]], x=[sample1["pose"][1], sample2["pose"][1]], **connect_style)
            connect_style["showlegend"] = False

        if self.shortest_path is not None:
            x_points = [sample[0] for sample in self.shortest_path]
            y_points = [sample[1] for sample in self.shortest_path]

            fig.add_scatter(y=x_points, x=y_points, showlegend = True, name="Path Found", line=dict(color="red", width=5))

        fig.update_xaxes({"autorange": True})
        fig.update_yaxes({"autorange": "reversed"})

        fig.update_layout({"title": {"text": "Connectivity Trees"}},
                paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

        if save:
            with open(PROJECT_PATH+"dashboard/data/mp.pickle", "wb") as file:
                pickle.dump(fig, file)
        else:
            fig.show()

########################### SETTERS AND GETTERS BELOW THIS POINT ###########################
    @property
    def grid_x_length(self):
        return self._grid_x_length

    @grid_x_length.setter
    def grid_x_length(self, val):
        assert isinstance(val, (float, int)),\
        f"grid_x_length must be type float or int and is type {type(val)}"
        assert val > 0, f"grid_x_length  must be positive they is {val}"
        self._grid_x_length = val

    @property
    def grid_y_length(self):
        return self._grid_y_length

    @grid_y_length.setter
    def grid_y_length(self, val):
        assert isinstance(val, (float, int)),\
                f"grid_y_length must be type float or int and is type {type(val)}"
        assert val > 0, f"grid_y_length  must be positive they is {val}"
        self._grid_y_length = val

    @property
    def obstacle(self):
        return self._obstacle

    @obstacle.setter
    def obstacle(self, obstacle):
        assert isinstance(obstacle, Obstacle),\
                f"obstacle must be of type Obstacle and is {type(obstacle)}"
        self._obstacle = obstacle

    @property
    def step_size(self):
        return self._step_size

    @step_size.setter
    def step_size(self, val):
        assert isinstance(val, (float, int)),\
                f"step_size must be type float or int and is type {type(val)}"
        assert val > 0, f"step_size must be positive they is {val}"
        self._step_size = val

    @property
    def search_size(self):
        return self._search_size

    @search_size.setter
    def search_size(self, val):
        assert isinstance(val, (float, int)),\
                f"search_size must be type float or int and is type {type(val)}"
        assert val > 0, f"search_size must be positive they is {val}"
        self._search_size = val

        self.samples = {}
        self.connect_trees = {}
        self.x_sorted = SortedDict({})
        self.y_sorted = SortedDict({})

    @property
    def n_samples(self):
        return self._n_samples

    @n_samples.setter
    def n_samples(self, val):
        self._n_samples = val
