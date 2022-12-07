import plotly.graph_objects as go
import pickle
import sys
import time
from sortedcontainers import SortedDict
from abc import ABC, abstractmethod
from robot_brain.obstacle import Obstacle
from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH 

from robot_brain.state import State


class MotionPlanner(ABC):
    """
    Motion planner that finds a start to target position for pushing and driving tasks.
    """
    def __init__(self, grid_x_length: float, grid_y_length: float, obstacle: Obstacle, step_size: float, search_size: float):

        self.grid_x_length = grid_x_length
        self.grid_y_length = grid_y_length
        self.obstacle = obstacle
        
        assert step_size < search_size, f"step size must be smaller than search size, step_size={step_size}, search_size={search_size}"
        self.step_size = step_size
        self.search_size = search_size
        self.start_time_search = 0

        self.source_tree_key = 0
        self.target_tree_key = 1

        self.samples = {}
        self.shortest_paths = SortedDict({})
        self.x_sorted = SortedDict({})
        self.y_sorted = SortedDict({})
        self.n_samples = 0

    @abstractmethod
    def setup(self, start_sample, target_sample):
        """ initialise the start and target samples. """
        pass

    @abstractmethod
    def search(self, start: State, target: State) -> tuple:
        """ search for a path between start and target state, raises error if no path can be found. """
        pass

    @abstractmethod
    def create_random_sample(self):
        """ Randomly generates a sample in free, movable or unknown space. """
        pass

    @abstractmethod
    def add_sample(self, sample: list, prev_key: int, cost_to_source: float) -> int:
        """ adds sample to all existing samples. """
        pass

    @abstractmethod
    def project_to_connectivity_graph(self, sample: list, project_to_sample_key: int) -> list:
        """ projects the sample closer to the closest existing sample in the connectivity graphs. """
        pass

    @abstractmethod
    def check_connecitvity(self, sample1: tuple, sample2: tuple) -> bool:
        """ check if 2 samples can be connected using a local planner. """
        pass

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

            temp_dist = self.distance(sample, self.samples[key]["pos"])
            if temp_dist < closest_distance:
                closest_sample_key = key
                closest_distance = temp_dist

        if closest_sample_key is None:
            raise ValueError("could not found a closest sample, which should be impossible")
        else:
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

        if planning_time > 0.5:
            raise StopIteration("It takes to long to find a path, halt.") 
        else:
            if len(self.shortest_paths) < 10: 
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

    def visualise(self, save=True, shortest_path=None):
        """ Visualise the connectivity graph. """

        fig = go.Figure()

        source_style = dict(showlegend = True, name="Source Tree", legendgroup=1, line=dict(color="blue"))
        target_style = dict(showlegend = True, name="Target Tree", legendgroup=2, line=dict(color="green"))
        connect_style = dict(showlegend = True, name="Connect Trees", legendgroup=3, line=dict(color="orange"))

        # loop through samples
        for sample in self.samples.values():

            prev_sample = self.samples[sample["prev_sample_key"]]["pos"]

            if sample["in_tree"] == self.source_tree_key:
                fig.add_scatter(y=[prev_sample[0], sample["pos"][0]], x=[prev_sample[1], sample["pos"][1]], **source_style)
                source_style["showlegend"] = False

            elif sample["in_tree"] == self.target_tree_key:
                fig.add_scatter(y=[prev_sample[0], sample["pos"][0]], x=[prev_sample[1], sample["pos"][1]], **target_style)
                target_style["showlegend"] = False
            else:
                raise ValueError(f"in_tree in unknown and is {sample['in_tree']}")

        for value in self.shortest_paths.values():
            sample1 = self.samples[value["sample1_key"]]
            sample2 = self.samples[value["sample2_key"]]
            fig.add_scatter(y=[sample1["pos"][0], sample2["pos"][0]], x=[sample1["pos"][1], sample2["pos"][1]], **connect_style)
            connect_style["showlegend"] = False

        if shortest_path is not None:
            x_points = [sample[0] for sample in shortest_path]
            y_points = [sample[1] for sample in shortest_path]
                
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
        assert isinstance(val, (float, int)), f"grid_x_length must be type float or int and is type {type(val)}"
        assert val > 0, f"grid_x_length  must be positive they is {val}"
        self._grid_x_length = val

    @property
    def grid_y_length(self):
        return self._grid_y_length

    @grid_y_length.setter
    def grid_y_length(self, val):
        assert isinstance(val, (float, int)), f"grid_y_length must be type float or int and is type {type(val)}"
        assert val > 0, f"grid_y_length  must be positive they is {val}"
        self._grid_y_length = val

    @property
    def obstacle(self):
        return self._obstacle

    @obstacle.setter
    def obstacle(self, obstacle):
        assert isinstance(obstacle, Obstacle), f"obstacle must be of type Obstacle and is {type(obstacle)}"
        self._obstacle = obstacle 

    @property
    def step_size(self):
        return self._step_size

    @step_size.setter
    def step_size(self, val):
        assert isinstance(val, (float, int)), f"step_size must be type float or int and is type {type(val)}"
        assert val > 0, f"step_size must be positive they is {val}"
        self._step_size = val

    @property
    def search_size(self):
        return self._search_size

    @search_size.setter
    def search_size(self, val):
        assert isinstance(val, (float, int)), f"search_size must be type float or int and is type {type(val)}"
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


