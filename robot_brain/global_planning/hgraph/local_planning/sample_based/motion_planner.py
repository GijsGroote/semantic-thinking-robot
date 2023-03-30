from abc import ABC, abstractmethod
from typing import Tuple
import pickle
import sys
import time
import warnings
import plotly.graph_objects as go
from sortedcontainers import SortedDict
import numpy as np

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.obstacle import Obstacle, FREE, UNMOVABLE, MOVABLE, UNKNOWN
from robot_brain.global_variables import FIG_BG_COLOR, PROJECT_PATH
from robot_brain.state import State
from robot_brain.global_planning.hgraph.local_planning.graph_based.rectangle_obstacle_path_estimator\
        import RectangleObstaclePathEstimator
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_obstacle_path_estimator\
        import CircleObstaclePathEstimator
from robot_brain.global_planning.hgraph.local_planning.graph_based.path_estimator import PathEstimator
from helper_functions.geometrics import to_interval_zero_to_two_pi, to_interval_min_pi_to_pi

### TODO: this motion planner is nice and all that but..
""" 3 issues:
    - adding samples from path estimation (without blocking objects) must not start sampling. Just take the samples from path estimation
     an exeption is the nonholonomic constaints, the it would be okey. but otherwise, just dont
     - appending samples from path estimation can trigger a Keyerror self.samples[prev_key] while prev_key = None
     - samples can be projected toward outside the grid
     - reusing a motion planner fails stuff, because the are not properly cleared
"""
class MotionPlanner(ABC):
    """
    Motion planner that finds a start to target position for pushing and driving tasks.
    """
    def __init__(self, grid_x_length: float,
            grid_y_length: float,
            obstacle: Obstacle,
            step_size: float,
            search_size: float,
            path_estimator: PathEstimator,
            include_orien: bool):

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

        self.include_orien = include_orien


        if include_orien:
            self.orien_sorted = SortedDict({})

        if isinstance(path_estimator, PathEstimator):
            if isinstance(obstacle.properties, CylinderObstacle):
                assert isinstance(path_estimator, CircleObstaclePathEstimator),\
                    "obstacle is CylinderObstacle, conf_grid_map should be of "\
                    f"type CircleObstaclePathEstimator and is {type(path_estimator)}"
            elif isinstance(obstacle.properties, BoxObstacle):
                assert isinstance(path_estimator, RectangleObstaclePathEstimator),\
                    "obstacle is BoxObstacle, conf_grid_map should be of type"\
                    f" RectangleObstaclePathEstimator and is {type(path_estimator)}"
            self.path_estimator = path_estimator
        else:
            raise ValueError("Incorrect or No PathEstimator provided")

        # reset variables that change every search
        self.reset()

    def reset(self):
        """ clear al samples that change during a search. """

        self.samples = {}
        self.shortest_paths = SortedDict({})
        self.shortest_path = None
        self.x_sorted = SortedDict({})
        self.y_sorted = SortedDict({})
        self.n_samples = 0

        if self.include_orien:
            self.orien_sorted = SortedDict({})

    def setup_source(self, source_sample: np.ndarray | tuple | list):

        assert isinstance(source_sample, (np.ndarray, tuple, list)), \
            "start- sample is not a type tuple, list or np.array which it should be."
        if len(source_sample) == 2:
            source_sample = np.array([source_sample[0], source_sample[1], 0])
        assert len(source_sample) == 3, \
                f"start- and target sample should have length 3 and have lengths {len(source_sample)} and {len(target_sample)}"

        self.samples.clear()
        self.x_sorted.clear()
        self.y_sorted.clear()
        if self.include_orien:
            self.orien_sorted.clear()

        # add negligleble amount to make positions hashable if initial position is 0
        if source_sample[0] == 0:
            source_sample[0] = float(6e-9)
        if source_sample[1] == 0:
            source_sample[1] = float(5e-9)
        if self.include_orien and source_sample[2] == 0:
            source_sample[2] = float(4e-9)

        source_sample[2] = to_interval_zero_to_two_pi(source_sample[2])

        # add start and target samples
        self.samples[self.source_tree_key] = {
                "pose": [source_sample[0], source_sample[1], source_sample[2]],
                "cost_to_source": 0,
                "prev_sample_key": self.source_tree_key,
                "next_sample_keys": [],
                "in_tree": self.source_tree_key,
                "add_node": []}


        self.n_samples = 1

        self.x_sorted = SortedDict({source_sample[0]: self.source_tree_key})
        self.y_sorted = SortedDict({source_sample[1]: self.source_tree_key})


    def setup_target(self, target_sample: np.ndarray | tuple | list):


        assert isinstance(target_sample, (np.ndarray, tuple, list)), \
            "target sample is not a type tuple, list or np.array which it should be."
        if len(target_sample) == 2:
            target_sample = np.array([target_sample[0], target_sample[1], 0])
        assert len(target_sample) == 3, \
                f"target sample should have length 3 and have lengths {len(source_sample)} and {len(target_sample)}"


        if target_sample[0] == 0:
            target_sample[0] = float(3e-9)
        if target_sample[1] == 0:
            target_sample[1] = float(2e-9)
        if  self.include_orien and target_sample[2] == 0:
            target_sample[2] = float(1e-9)
        target_sample[2] = to_interval_zero_to_two_pi(target_sample[2])

        self.samples[self.target_tree_key] = {
                "pose": [target_sample[0], target_sample[1], target_sample[2]],
                "cost_to_source": 0,
                "prev_sample_key": self.target_tree_key,
                "next_sample_keys": [],
                "in_tree": self.target_tree_key,
                "add_node": []}

        self.n_samples = 2

        self.x_sorted[target_sample[0]] = self.target_tree_key
        self.y_sorted[target_sample[1]] = self.target_tree_key


    def search_path(self, source: State, target: State) -> Tuple[list, list]:
        """ search for a path between source and target state. """

        source_sample = source.get_2d_pose()
        target_sample = target.get_2d_pose()

        self.path_estimator.update()
        self.reset()

        self.setup_source(source_sample)
        self.start_time_search = time.time()

        # add samples from path estimation
        self._add_path_estimator_samples(source_sample, target_sample)
        self.setup_target(target_sample)
        close_samples_keys = self._get_closeby_sample_keys(target_sample, self.search_size)
        self._rewire_close_samples_and_connect_trees(self.target_tree_key, close_samples_keys)
        print(f'adding pe samples takes {time.time() -self.start_time_search } seconds')

        # return path if it exist and goes through free space only
        if len(self.shortest_paths) > 0:
            (path, add_node_list) = self._extract_shortest_path()
            if len(add_node_list) == 0:
                self.shortest_path = path
                return (path, add_node_list)

        print(f'lets search a little more this already took {time.time() -self.start_time_search } seconds')

        # while keep searching:
        while not self._stop_criteria_test():

            # generate random sample
            sample_rand = self._create_random_sample()

            # find closest sample
            sample_closest_key = self._get_closest_sample_key(sample_rand)

            # project it to existing samples
            if self._distance(self.samples[sample_closest_key], sample_rand) > self.step_size:
                sample_new = self._project_to_connectivity_graph(sample_rand, sample_closest_key)
                # TODO: the project_to_connectivity graph can project samples outside grid if the
                # xgridlength and ygridlength is not equal
                if not self.sample_outside_grid(sample_new):
                    continue
            else:
                sample_new = sample_rand


            in_space_id = self.path_estimator.occupancy(np.array(sample_new))
            if in_space_id == UNMOVABLE: # obstacle space, abort sample
                print('sample in unmovable space')
                continue

            # find closeby samples and connect new sample to cheapest sample
            close_samples_keys = self._get_closeby_sample_keys(sample_new, self.search_size)

            try:
                sample_new_key = self._connect_to_cheapest_sample(sample_new, close_samples_keys, in_space_id)
            except [AssertionError, ValueError] as exc:
                print('error occured with sample')
                warnings.warn(f"emerged Error during planning, discard sample: {exc}")
                continue

            # rewire close samples lowering their cost, connect to other tree if possible
            self._rewire_close_samples_and_connect_trees(sample_new_key, close_samples_keys)

        (path, add_node_list)  = self._extract_shortest_path()
        self.shortest_path = path

        print('thjis visualisation should be a complete path after mp')
        self.visualise(save=False)
        self.path_estimator.visualise(save=False)

        return (path, add_node_list)

    def _add_path_estimator_samples(self, source_sample, target_sample):
        """ convert samples from path estimation to motion planner. """
        assert self.search_size > self.path_estimator.cell_size*1.5,\
                f"the motion planner search size: {self.search_size} is smaller "\
                f"than conf_grid_map.cell_size * 1.5: {self.path_estimator.cell_size*1.5}"

        shortest_path = self.path_estimator.search_path(source_sample, target_sample)
        # shortest_path = shortest_path[1:-1]
        # do not add if shortest path does go though movable or unknown space
        for sample in shortest_path:
            if self.path_estimator.occupancy(np.array(sample)) != FREE:
                print('Do not add shortest path that goes through movable of unknown space')
                return


        for sample_new in shortest_path:
            # add negligble amount making x and y coordinates unique
            # NOTE: At the edges, this small additional could put the sample outside of the
            # environment, just as it could 'pass' 2*pi
            sample_new += np.abs(np.random.normal(0, 0.0000001, np.asarray(sample_new).shape))
            if sample_new.shape == (2,):
                sample_new = np.array([sample_new[0], sample_new[1], 0])

            in_space_id = self.path_estimator.occupancy(np.array(sample_new))

            close_samples_keys = self._get_closeby_sample_keys(sample_new, self.search_size)

            try:
                sample_new_key = self._connect_to_cheapest_sample(sample_new, close_samples_keys, in_space_id)
            except ValueError:
                continue

    @abstractmethod
    def _create_random_sample(self):
        """ Randomly generates a sample in free, movable or unknown space. """

    def _add_sample(self, sample: list, prev_key: int, cost_to_source, add_node: bool) -> int:
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

        key = self._create_unique_id()
        self.samples[prev_key]["next_sample_keys"].append(key)

        self.samples[key] = {
                "pose": sample,
                "cost_to_source": cost_to_source,
                "prev_sample_key": prev_key,
                "next_sample_keys": [],
                "in_tree": self.samples[prev_key]["in_tree"],
                "add_node": add_node}

        self.x_sorted[sample[0]] = key
        self.y_sorted[sample[1]]= key

        if self.include_orien:
            self.orien_sorted[sample[2]]= key

        self.n_samples += 1

        return key

    @abstractmethod
    def _project_to_connectivity_graph(self, sample: list, project_to_sample_key: int) -> list:
        """ projects the sample closer to the closest existing sample in the connectivity graphs. """

    @abstractmethod
    def _check_connecitvity(self, sample1: tuple, sample2: tuple) -> bool:
        """ check if 2 samples can be connected using a local planner. """

    @abstractmethod
    def _extract_shortest_path(self) -> Tuple[list, list]:
        """ Finds the shortest path after sampling. """

    @abstractmethod
    def _rewire_close_samples_and_connect_trees(self, sample_key, close_samples_keys: list):
        """ rewire closeby samples if that lowers the cost for that sample,
        connect to the cheapest sample from the other tree if possible. """

    @abstractmethod
    def _connect_to_cheapest_sample(self, sample: list, close_samples_keys: list, in_space_id: int) -> int:
        """ finds and connect to the closeby sample which gives the cheapest path. """

    def _get_closest_sample_key(self, sample: list) -> int:
        """ Search for closest points in x and y """

        # speed up, only compare closeby samples before comparing to all existing samples
        test_closest_keys = self._get_closeby_sample_keys(sample, 10*self.grid_x_length/self.n_samples)

        if len(test_closest_keys) > 0:
            closest_keys = test_closest_keys
        else:
            test_closest_keys = self._get_closeby_sample_keys(sample, 100*self.grid_x_length/self.n_samples)
            if len(test_closest_keys) > 0:
                closest_keys = test_closest_keys
            else:
                closest_keys = self.samples.keys()

        closest_sample_key = None
        closest_distance = sys.float_info.max
        for key in closest_keys:

            temp_dist = self._distance(sample, self.samples[key])
            if temp_dist < closest_distance:
                closest_sample_key = key
                closest_distance = temp_dist

        # if closest_sample_key is None:
        #     raise ValueError("could not found a closest sample, which should be impossible")
        return closest_sample_key

    def _distance(self, sample1: dict | list, sample2: dict | list) -> float:
        """ return euclidean distance, add orientation cost of flag is set. """
        if isinstance(sample2, dict):
            sample2 = sample2["pose"]
        if isinstance(sample1, dict):
            sample1 = sample1["pose"]

        xy_distance = np.linalg.norm([sample1[0] - sample2[0],\
                    sample1[1] - sample2[1]])

        if self.include_orien:
            orien_cost = np.abs(to_interval_min_pi_to_pi(to_interval_zero_to_two_pi(sample1[2])\
                        -to_interval_zero_to_two_pi(sample2[2])))
            return xy_distance + orien_cost

        else:
            return float(xy_distance)

    def _get_closeby_sample_keys(self, sample: list, radius: float) -> set:
        """ return the keys of samples which are less than 2*radius manhattan distance to sample. """
        x_keys = set()
        for x_key in self.x_sorted.irange(sample[0]-radius, sample[0]+radius):
            x_keys.add(self.x_sorted[x_key])

        y_keys = set()
        for y_key in self.y_sorted.irange(sample[1]-radius, sample[1]+radius):
            y_keys.add(self.y_sorted[y_key])

        return x_keys.intersection(y_keys)

    def _calculate_path_cost(self, sample1: dict, sample2: dict) -> float:
        """ calculate the cost for a path from source to target. """
        return sample1["cost_to_source"] + sample2["cost_to_source"] + self._distance(sample1, sample2)

    @abstractmethod
    def _stop_criteria_test(self) -> bool:
        """ test is the shortest path converged. """

    def sample_outside_grid(self, sample: np.ndarray) -> bool:
        """ return True if the sample is inside the grid. """
        return np.abs(sample[0]) < self.grid_x_length/2 and np.abs(sample[1]) < self.grid_y_length/2

    def _create_unique_id(self) -> int:
        """ creates and returns a unique id. """
        unique_id = len(self.samples)

        while unique_id in self.samples:
            unique_id += 1

        return unique_id

    def visualise(self, save=True):
        """ Visualise the connectivity graph. """

        fig = go.Figure()

        source_style = dict(showlegend = True,
                name="Source Tree", legendgroup=1, line=dict(color="blue"), hoverinfo="x+y+text")
        target_style = dict(showlegend = True,
                name="Target Tree", legendgroup=2, line=dict(color="green"), hoverinfo="x+y+text")
        connect_style = dict(showlegend = True,
                name="Connect Trees", legendgroup=3, line=dict(color="orange"), hoverinfo="x+y+text")

        # fig.add_scatter(y=[0,0],
        #                 x=[0,0], **source_style)
        # fig.add_scatter(y=[0,0],
        #                 x=[0,0], **target_style)
        # fig.add_scatter(y=[0,0],
        #                 x=[0,0], **connect_style)

        # loop through samples
        for sample in self.samples.values():
            prev_sample = self.samples[sample["prev_sample_key"]]
            prev_sample_pose = prev_sample["pose"]
            hover_text = [f"cost: {prev_sample['cost_to_source']}", f"cost: {sample['cost_to_source']}"]


            if sample["in_tree"] == self.source_tree_key:
                source_style["hovertext"] = hover_text
                fig.add_scatter(y=[prev_sample_pose[0], sample["pose"][0]],
                        x=[prev_sample_pose[1], sample["pose"][1]], **source_style)

                source_style["showlegend"] = False

            elif sample["in_tree"] == self.target_tree_key:
                target_style["hovertext"] = hover_text
                fig.add_scatter(y=[prev_sample_pose[0], sample["pose"][0]], x=[prev_sample_pose[1], sample["pose"][1]], **target_style)

                target_style["showlegend"] = False

            else:
                raise ValueError(f"in_tree in unknown and is {sample['in_tree']}")

        for short_path in self.shortest_paths.values():
            sample1 = self.samples[short_path["sample1_key"]]
            sample2 = self.samples[short_path["sample2_key"]]


            cost_path = self._calculate_path_cost(sample1, sample2)

            connect_style["hovertext"] = f"cost path: {cost_path}"
            fig.add_scatter(y=[sample1["pose"][0], sample2["pose"][0]],
                    x=[sample1["pose"][1], sample2["pose"][1]], **connect_style)
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
