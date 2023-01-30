import random
import sys
from typing import Tuple
import time
import math
from sortedcontainers import SortedDict
import numpy as np

from robot_brain.obstacle import Obstacle
from robot_brain.global_planning.hgraph.local_planning.sample_based.motion_planner import MotionPlanner
from robot_brain.global_variables import KNOWN_OBSTACLE_COST, UNKNOWN_OBSTACLE_COST
from robot_brain.global_planning.hgraph.local_planning.graph_based.path_estimator import PathEstimator

from robot_brain.exceptions import PlanningTimeElapsedException
from helper_functions.geometrics import to_interval_zero_to_two_pi

class DriveMotionPlanner(MotionPlanner):
    """ Motion planner, using a double rapid randomly tree star (RRT*)
    to search a path for the obstacle to track. """

    def __init__(self,
        grid_x_length: float,
        grid_y_length: float,
        obstacle: Obstacle,
        step_size: float,
        search_size: float,
        path_estimator: PathEstimator,
        include_orien=False):

        MotionPlanner.__init__(self, grid_x_length, grid_y_length, obstacle,
                step_size, search_size, path_estimator, include_orien)

        self.start_time = None

    def setup(self, source_sample: np.ndarray | tuple | list, target_sample: np.ndarray | tuple | list):

        assert isinstance(source_sample, (np.ndarray, tuple, list)) and isinstance(target_sample, (np.ndarray, tuple, list)), \
            "start- or target sample is not a type tuple, list or np.array which it should be."
        if len(source_sample) == 2:
            source_sample = np.array([source_sample[0], source_sample[1], 0])
        if len(target_sample) == 2:
            target_sample = np.array([target_sample[0], target_sample[1], 0])
        assert len(source_sample) == 3 and len(target_sample) == 3, \
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

        if target_sample[0] == 0:
            target_sample[0] = float(3e-9)
        if target_sample[1] == 0:
            target_sample[1] = float(2e-9)
        if  self.include_orien and target_sample[2] == 0:
            target_sample[2] = float(1e-9)
        source_sample[2] = to_interval_zero_to_two_pi(source_sample[2])
        target_sample[2] = to_interval_zero_to_two_pi(target_sample[2])

        # add start and target samples
        self.samples[self.source_tree_key] = {
                "pose": [source_sample[0], source_sample[1], source_sample[2]],
                "cost_to_source": 0,
                "prev_sample_key": self.source_tree_key,
                "next_sample_keys": [],
                "in_tree": self.source_tree_key,
                "add_node": []}

        self.samples[self.target_tree_key] = {
                "pose": [target_sample[0], target_sample[1], target_sample[2]],
                "cost_to_source": 0,
                "prev_sample_key": self.target_tree_key,
                "next_sample_keys": [],
                "in_tree": self.target_tree_key,
                "add_node": []}

        self.n_samples = 2

        self.x_sorted = SortedDict({source_sample[0]: self.source_tree_key, target_sample[0]: self.target_tree_key})
        self.y_sorted = SortedDict({source_sample[1]: self.source_tree_key, target_sample[1]: self.target_tree_key})

    def _create_random_sample(self) -> list:
        """ randomly generate sample inside grid boundaries. """
        x_rand = random.uniform(-self.grid_x_length/2, self.grid_x_length/2)
        y_rand = random.uniform(-self.grid_y_length/2, self.grid_y_length/2)

        if self.include_orien:
            return [x_rand, y_rand, random.uniform(0, 2*math.pi)]
        else:
            return [x_rand, y_rand]

    def _check_connecitvity(self, sample1: tuple, sample2: tuple) -> bool:
        """ check if 2 samples can be connected using a local planner. """
        # TODO: The drive motion planner is always able to connect two samples
        # As long as you pick only a holonomic robot
        return True

    def _connect_to_cheapest_sample(self, sample: list, close_samples_keys: list, in_space_id: int) -> int:
        """ finds and connect to the closeby sample which gives the cheapest path. """

        closest_sample_total_cost = sys.float_info.max
        closest_sample_add_node_cost = 0
        closest_sample_key = None

        for close_sample_key in close_samples_keys:

            close_sample = self.samples[close_sample_key]

            # add cost or an additional subtask
            close_sample_add_node_cost = 0
            if in_space_id == 2: # movable space
                if self.path_estimator.occupancy(np.array(close_sample["pose"])) != 2:
                    close_sample_add_node_cost = KNOWN_OBSTACLE_COST

            elif in_space_id == 3: # unkown space
                if self.path_estimator.occupancy(np.array(close_sample["pose"])) != 3:
                    close_sample_add_node_cost = UNKNOWN_OBSTACLE_COST

            close_sample_total_cost = close_sample["cost_to_source"] +\
                    self._distance(close_sample["pose"], sample) + close_sample_add_node_cost

            

            if close_sample_total_cost < closest_sample_total_cost:
                closest_sample_add_node_cost = close_sample_add_node_cost
                closest_sample_total_cost = close_sample_total_cost
                closest_sample_key = close_sample_key

        add_node = False
        if closest_sample_add_node_cost > 0:
            add_node = True

        # add new sample
        return self._add_sample(sample, closest_sample_key, closest_sample_total_cost, add_node)

    def _rewire_close_samples_and_connect_trees(self, sample_key, close_samples_keys: list):
        """ rewire closeby samples if that lowers the cost for that sample,
        connect to the cheapest sample from the other tree if possible. """

        sample = self.samples[sample_key]

        # lower cost for samples around new sample and connect to the other tree
        cheapest_path_cost = sys.float_info.max
        cheapest_path_cost_sample_key = None

        # closeby samples are updated if the cost can be lowered
        for temp_close_sample_key in close_samples_keys:

            temp_close_sample = self.samples[temp_close_sample_key]

            if temp_close_sample["in_tree"] == sample["in_tree"]:

                # check if cost can be lowered for closeby samples
                cost_to_close_sample = sample["cost_to_source"] + self._distance(temp_close_sample, sample)
                if cost_to_close_sample < temp_close_sample["cost_to_source"]:

                    # rewire and update all next samples cost
                    self.samples[temp_close_sample["prev_sample_key"]]["next_sample_keys"].remove(temp_close_sample_key)
                    temp_close_sample["prev_sample_key"] = sample_key
                    sample["next_sample_keys"].append(temp_close_sample_key)
                    self._update_cost_sample(temp_close_sample_key, temp_close_sample, cost_to_close_sample)

            else:
                # find lowest cost to connect both trees
                temp_path_cost = self._calculate_path_cost(sample, temp_close_sample)

                if temp_path_cost < cheapest_path_cost:

                    cheapest_path_cost = temp_path_cost
                    cheapest_path_cost_sample_key = temp_close_sample_key

        # found a path to connect both trees
        if (cheapest_path_cost_sample_key is not None and
                self._distance(sample, self.samples[cheapest_path_cost_sample_key]) < self.step_size):
            self.shortest_paths[cheapest_path_cost] = {"sample1_key": sample_key,
                    "sample2_key": cheapest_path_cost_sample_key}

    def _project_to_connectivity_graph(self, sample: list, project_to_sample_key: int) -> list:
        """ projects the sample closer to the closest existing sample in the connectivity graphs. """

        # TODO: using vectors speeds up, convert sin/cos calculation to vector calculation
        project_to_sample = self.samples[project_to_sample_key]

        if self._distance(project_to_sample, sample) > self.step_size:

            theta = np.arctan2(sample[1]-project_to_sample["pose"][1], sample[0]-project_to_sample["pose"][0])
            x_new = project_to_sample["pose"][0] + np.cos(theta) * self.step_size
            y_new = project_to_sample["pose"][1] + np.sin(theta) * self.step_size

            orien_new = 0
            if self.include_orien:
                orien_new = to_interval_zero_to_two_pi(sample[2]-project_to_sample["pose"][2])

            return [x_new, y_new, orien_new]
        else:
            return sample

    def _update_cost_sample(self, sample_key: int, sample: dict, new_cost: float):
        """ update sample cost for sample and next samples, including cost for shortest paths. """

        # update cost for this sample,
        for next_sample_key in sample["next_sample_keys"]:

            next_sample = self.samples[next_sample_key]
            next_sample_new_cost = new_cost + self._distance(next_sample, sample)

            if next_sample["in_tree"] == sample["in_tree"]:

                # recursively update new costs
                self._update_cost_sample(next_sample_key, next_sample, next_sample_new_cost)

            else:
                # update the existing shortest path found shortest path
                shortest_path_key = sample["cost_to_source"] + self._distance(sample, next_sample) + next_sample["cost_to_source"]
                self.shortest_paths.pop(shortest_path_key)

                new_shortest_path_key = next_sample_new_cost + next_sample["cost_to_source"]
                self.shortest_paths[new_shortest_path_key] = {"sample1_key": next_sample_key, "sample2_key": sample_key}

    def _stop_criteria_test(self) -> bool:
        """ test is the shortest path converged. """

        planning_time = time.time() - self.start_time_search

        if planning_time < 0.5: # be picky
            return len(self.shortest_paths) > 10 and self.shortest_paths.peekitem(0)[0] * 1.05 > self.shortest_paths.peekitem(9)[0]
        elif planning_time < 1: # be less picky
            return len(self.shortest_paths) > 5 and self.shortest_paths.peekitem(0)[0] * 1.10 > self.shortest_paths.peekitem(4)[0]
        elif len(self.shortest_paths) > 0: # beg for anything
            return True
        else:
            raise PlanningTimeElapsedException("It takes to long to find a path, halt.")


    def _extract_shortest_path(self) -> Tuple[list, list]:
        """ Finds the shortest path after sampling. """
        if len(self.shortest_paths) == 0:
            raise ValueError("start to target tree is not connected")

        shortest_path_samples = self.shortest_paths.values()[0]
        sample1 = self.samples[shortest_path_samples["sample1_key"]]
        sample2 = self.samples[shortest_path_samples["sample2_key"]]

        if sample1["in_tree"] == self.source_tree_key:
            source_sample = sample1
            target_sample = sample2
        else:
            target_sample = sample1
            source_sample = sample2

        reversed_path = []

        add_node_list = []
        while source_sample["prev_sample_key"] != self.source_tree_key:
            if source_sample["add_node"]:
                add_node_list.append(source_sample["pose"])

            reversed_path.append(source_sample["pose"])
            source_sample = self.samples[source_sample["prev_sample_key"]]

        reversed_path.append(self.samples[self.source_tree_key]["pose"])
        path = list(reversed(reversed_path))

        while target_sample["prev_sample_key"] != self.target_tree_key:
            if target_sample["add_node"]:
                add_node_list.append(target_sample["pose"])

            path.append(target_sample["pose"])
            target_sample = self.samples[target_sample["prev_sample_key"]]

        path.append(self.samples[self.target_tree_key]["pose"])

        return (path, add_node_list)
