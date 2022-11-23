import numpy as np
import plotly.graph_objects as go
import plotly.express as px
import pickle
import sys
from sortedcontainers import SortedDict
from bisect import bisect
from abc import ABC, abstractmethod
from robot_brain.obstacle import Obstacle
from robot_brain.global_planning.hgraph.local_planning.graph_based.rectangular_robot_configuration_grid_map import RectangularRobotConfigurationGridMap

from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH 
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_robot_configuration_grid_map import CircleRobotConfigurationGridMap 
from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle 

from robot_brain.state import State


class MotionPlanner(ABC):
    """
    Doubly Rapid Random Tree star (RRT*) motion planner.
    """
    def __init__(self, grid_x_length: float, grid_y_length: float, obstacle: Obstacle, step_size: float, search_size: float):

        assert isinstance(grid_x_length, (float, int)), f"grid_x_length must be type float or int and is type {type(grid_x_length)}"
        assert isinstance(grid_y_length, (float, int)), f"grid_y_length must be type float or int and is type {type(grid_y_length)}"
        assert grid_x_length > 0 and grid_y_length > 0, f"both grid_x_length and grid_y_length must be positive they are {grid_x_length} and {grid_y_length}"

        self._grid_x_length = grid_x_length
        self._grid_y_length = grid_y_length

        self.obstacle = obstacle
        self.step_size = step_size
        self.search_size = search_size

        self.source_tree_key = 0
        self.target_tree_key = 1

        self.samples = {}
        self.connect_trees = {}
        self.x_sorted = SortedDict({})
        self.y_sorted = SortedDict({})

    @abstractmethod
    def setup(self, start_sample, target_sample):
        """ initialise the connectivity graphs with the start and target samples. """
        pass

    @abstractmethod
    def search(self, start: State, target: State) -> list:
        """ search for a path between start and target state, raises error if no path can be found. """
        pass

    @abstractmethod
    def create_random_sample(self):
        """ Randomly generates a sample in free space. """
        pass

    @abstractmethod
    def check_connecitvity(self, sample1: tuple, sample2: tuple) -> bool:
        """ check if 2 samples can be connected using a local planner. """
        pass

    def add_sample(self, sample: list, prev_key: int, cost_to_source) -> int:
        """ adds sample to all existing samples. """
        pass

    def get_closest_sample_key(self, sample: list) -> int:
        """ Search for closest points in x and y """

        test_closest_keys = self.get_closeby_sample_keys(sample, 0.1)
        if len(test_closest_keys) > 0:
            closest_keys = test_closest_keys
        else:
            test_closest_keys = self.get_closeby_sample_keys(sample, 1)
            if len(test_closest_keys) > 0:
                closest_keys = test_closest_keys
            else:
                test_closest_keys = self.get_closeby_sample_keys(sample, 4)

                if len(test_closest_keys) > 0:
                    closest_keys = test_closest_keys
                else:
                    closest_keys = self.samples.keys()

        closest_sample_key = None
        closest_distance = sys.float_info.max
        for key in closest_keys:

            temp_dist = self.distance(sample, self.samples[key]["pos"])
            if temp_dist < closest_distance:
                closest_sample_key = key
                closest_distance = temp_dist

        return closest_sample_key

    def distance(self, sample1: list, sample2: list) -> float:
        """ returns euclidean distance between 2 samples. """
        return np.linalg.norm([sample1[0] - sample2[0], sample1[1] - sample2[1]])

    def get_closeby_sample_keys(self, sample: list, radius: float) -> set:
        """ return the keys of samples which are less than 2*radius manhattan distance to sample. """
        
        x_keys = set()
        for x_key in self.x_sorted.irange(sample[0]-radius, sample[0]+radius):
            x_keys.add(self.x_sorted[x_key])

        y_keys = set()
        for y_key in self.y_sorted.irange(sample[1]-radius, sample[1]+radius):
            y_keys.add(self.y_sorted[y_key])

        return x_keys.intersection(y_keys)

    def get_closest_samples(self, sorted_list: SortedDict, val: float, n: int) -> list:
        """ return a list with n sample keys closest to val in list. """

        assert sorted_list.__contains__(val), f"value {val} does not exist in the sorted list"

        # TODO: Check for index out of range 
        val_idx = sorted_list.index(val)

        high_idx = val_idx+1
        low_idx = val_idx-1

        close_keys = []
        
        for _ in range(n):

            high = sorted_list.peekitem(high_idx)[0]
            low = sorted_list.peekitem(low_idx)[0]

            if abs(high-val) < abs(low-val):
                close_keys.append(high)
                high_idx += 1
            else:
                close_keys.append(low)
                low_idx -= 1

        return close_keys

    def create_unique_id(self) -> int:
        """ creates and returns a unique id. """
        unique_id = len(self.samples)

        while unique_id in self.samples.keys():
            unique_id += 1

        return unique_id

    def visualise(self, save=False, shortest_path=None):
        """ Visualise the connectivity graph. """

        fig = go.Figure()

        source_style = dict(showlegend = True, name="Source Tree", legendgroup="Source Tree", line=dict(color="blue"))
        target_style = dict(showlegend = True, name="Target Tree", legendgroup="Target Tree", line=dict(color="green"))
        connect_style = dict(showlegend = True, name="Connect Trees", legendgroup="Connect Trees", line=dict(color="orange"))

        # loop through samples
        for sample in self.samples.values():

            prev_sample = self.samples[sample["prev_sample_key"]]["pos"]

            if sample["in_tree"] == self.source_tree_key:
                fig.add_scatter(y=[prev_sample[0], sample["pos"][0]], x=[prev_sample[1], sample["pos"][1]], **source_style)
                
                source_style["showlegend"] = False # this could be done better, remove this part. 

            elif sample["in_tree"] == self.target_tree_key:
                fig.add_scatter(y=[prev_sample[0], sample["pos"][0]], x=[prev_sample[1], sample["pos"][1]], **target_style)
                target_style["showlegend"] = False
            else:
                raise ValueError(f"in_tree in unknown and is {sample['in_tree']}")

        for (key, value) in self.connect_trees.items():
            sample1 = self.samples[key]
            sample2 = self.samples[value["connected_to"]]
            fig.add_scatter(y=[sample1["pos"][0], sample2["pos"][0]], x=[sample1["pos"][1], sample2["pos"][1]], **connect_style)
            connect_style["showlegend"] = False

        if shortest_path is not None:
            x_points = [sample[0] for sample in shortest_path]
            y_points = [sample[1] for sample in shortest_path]
                
            fig.add_scatter(y=x_points, x=y_points, **dict(showlegend = True, name="Connect Trees", line=dict(color="red", width=20)))
        
        fig.update_xaxes({"autorange": True})
        fig.update_yaxes({"autorange": "reversed"})

        fig.update_layout({"title": {"text": "Connectivity Trees"}},
                paper_bgcolor=FIG_BG_COLOR, plot_bgcolor=FIG_BG_COLOR)

        if save:
            with open(PROJECT_PATH+"dashboard/data/controller.pickle", "wb") as file:
                pickle.dump(fig, file)
        else:
            fig.show()

    def project_to_connectivity_graph(self, sample: tuple):
        """ Finds closest sample in connectivity graph and moves sample toward that sample. """
        raise NotImplementedError

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

    @property
    def connectivity_graph(self):
        return self._connectivity_graph

    @connectivity_graph.setter
    def connectivity_graph(self, conn_graph):
        self._connectivity_graph = conn_graph

