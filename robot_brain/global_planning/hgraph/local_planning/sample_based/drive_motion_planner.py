import numpy as np    
import random
import sys
import time
from sortedcontainers import SortedDict
import warnings

import plotly.express as px
from abc import abstractmethod
from robot_brain.obstacle import Obstacle
from robot_brain.global_planning.hgraph.local_planning.graph_based.rectangular_robot_configuration_grid_map import RectangularRobotConfigurationGridMap
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_robot_configuration_grid_map import CircleRobotConfigurationGridMap 
from robot_brain.state import State
from robot_brain.global_planning.hgraph.local_planning.sample_based.motion_planner import MotionPlanner
from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle 
from robot_brain.global_variables import KNOWN_OBSTACLE_COST, UNKNOWN_OBSTACLE_COST


class DriveMotionPlanner(MotionPlanner):
    """ Motion planner, using a double rapid randomly tree star (RRT*) to search a path for the obstacle to track. """

    def __init__(self, 
        grid_x_length: float,
        grid_y_length: float,
        obstacles: dict,
        obstacle: Obstacle,
        step_size: float,
        search_size: float):

        MotionPlanner.__init__(self, grid_x_length, grid_y_length, obstacle, step_size, search_size)

        if isinstance(obstacle.properties, CylinderObstacle):
            self.configuration_space_grid_map = CircleRobotConfigurationGridMap(
                cell_size=0.2,
                grid_x_length=grid_x_length,
                grid_y_length=grid_y_length,
                obstacles=obstacles,
                robot_cart_2d=obstacle.state.get_xy_position(),
                robot_radius=obstacle.properties.radius()) # radius or radius()???

        elif isinstance(obstacle.properties, BoxObstacle):
            self.configuration_space_grid_map = RectangularRobotConfigurationGridMap(
                cell_size=0.2,
                grid_x_length=grid_x_length,
                grid_y_length=grid_y_length,
                obstacles=obstacles,
                robot_cart_2d=obstacle.state.get_xy_position(),
                n_orientations= 36,
                robot_x_length=obstacle.properties.length(),
                robot_y_length=obstacle.properties.width())
        else:
            raise ValueError("The robot has an unknown obstacle")

    def setup(self, start_sample, target_sample):

        assert isinstance(start_sample, (np.ndarray, tuple, list)) and isinstance(target_sample, (np.ndarray, tuple, list)), \
            "start- or target sample is not a type tuple, list or np.array which it should be."
        assert len(start_sample) == 2 and len(target_sample) == 2, \
                f"start- and target sample should have length 2 and have lengths {len(start_sample)} and {len(target_sample)}"

        self.samples.clear()
        self.x_sorted.clear()
        self.y_sorted.clear()

        # add start and target samples
        self.samples[self.source_tree_key] = {
                "pos": [start_sample[0], start_sample[1]],
                "cost_to_source": 0,
                "prev_sample_key": self.source_tree_key,
                "next_sample_keys": [],
                "in_tree": self.source_tree_key}

        self.samples[self.target_tree_key] = {
                "pos": [target_sample[0], target_sample[1]],
                "cost_to_source": 0,
                "prev_sample_key": self.target_tree_key,
                "next_sample_keys": [],
                "in_tree": self.target_tree_key}

        self.n_samples = 2
        self.x_sorted = SortedDict({start_sample[0]: self.source_tree_key, target_sample[0]: self.target_tree_key})
        self.y_sorted = SortedDict({start_sample[1]: self.source_tree_key, target_sample[1]: self.target_tree_key})

    def search(self, start: State, target: State) -> list:
        """ search for a path between start and target state, raises error if no path can be found. """
        # TODO: this does not raise an error and it should after a number of iterations

        self.configuration_space_grid_map.update()
        # TODO:self.configuration_space_grid_map.shortest_path() # TODO: add samples to the RRT* planning algorithm 
        # before searching for a path

        self.setup(start.get_xy_position(), target.get_xy_position())

        start_time_search = time.time()

        # while keep_searching:
        while not self.paths_converged_test():

            if time.time() - start_time_search > 5:
                if len(self.shortest_paths) > 0:
                    return self.extract_shortest_path()
                else:
                    print('hey that has not pasth yet')




            # TODO: terminate if it takes to long and no path has been found

            # generate random sample
            sample_rand = self.create_random_sample()

            # find closest sample
            sample_closest_key = self.get_closest_sample_key(sample_rand)

            # project it to existing samples
            sample_new = self.project_to_connectivity_graph(sample_rand, sample_closest_key)

            in_space_id = self.configuration_space_grid_map.occupancy(np.array(sample_new))
            if in_space_id == 1: # obstacle space, abort sample
                continue

            # find closeby samples and connect new sample to cheapest sample
            close_samples_keys = self.get_closeby_sample_keys(sample_new, self.search_size)

            try: 
                sample_new_key = self.connect_to_cheapest_sample(sample_new, close_samples_keys, in_space_id)
            except AssertionError:
                warnings.warn("duplicate key found in sorted pose of sample")
                continue

            # rewire close samples lowering their cost, connect to other tree if possible 
            self.rewire_close_samples_and_connect_trees(sample_new_key, close_samples_keys)

        # visualisation could be removed
        path = self.extract_shortest_path()
        return path

    def create_random_sample(self) -> list:
        """ randomly generate sample inside grid boundaries. """
        x_rand = random.uniform(-self.grid_x_length/2, self.grid_x_length/2)
        y_rand = random.uniform(-self.grid_y_length/2, self.grid_y_length/2)

        return [x_rand, y_rand]

    def check_connecitvity(self, sample1: tuple, sample2: tuple) -> bool:
        """ check if 2 samples can be connected using a local planner. """
        # The drive motion planner is always able to connect two samples
        return True

    def connect_to_cheapest_sample(self, sample: list, close_samples_keys: list, in_space_id: int) -> int:
        """ finds and connect to the closeby sample which gives the cheapest path. """ 

        closest_sample_total_cost = sys.float_info.max
        closest_sample_key = None

        for close_sample_key in close_samples_keys:

            close_sample = self.samples[close_sample_key]

            # add cost or an additional subtask
            add_subtask_cost = 0
            if in_space_id == 2: # movable space
                if self.configuration_space_grid_map.occupancy(np.array(close_sample["pos"])) != 2:
                    add_subtask_cost = KNOWN_OBSTACLE_COST 

            elif in_space_id == 3: # unkown space
                if self.configuration_space_grid_map.occupancy(np.array(close_sample["pos"])) != 3:
                    add_subtask_cost = UNKNOWN_OBSTACLE_COST

            temp_total_cost = close_sample["cost_to_source"] + self.distance(close_sample["pos"], sample) + add_subtask_cost

            if temp_total_cost < closest_sample_total_cost:
                closest_sample_total_cost = temp_total_cost
                closest_sample_key = close_sample_key

            # add new sample
            return self.add_sample(sample, closest_sample_key, closest_sample_total_cost)


    def rewire_close_samples_and_connect_trees(self, sample_key, close_samples_keys: list):
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
                cost_to_close_sample = sample["cost_to_source"]+self.distance(temp_close_sample["pos"], sample["pos"])
                if cost_to_close_sample < temp_close_sample["cost_to_source"]:

                    # rewire and update all next samples cost
                    self.samples[temp_close_sample["prev_sample_key"]]["next_sample_keys"].remove(temp_close_sample_key)
                    temp_close_sample["prev_sample_key"] = sample_key
                    sample["next_sample_keys"].append(temp_close_sample_key)
                    self.update_cost_sample(temp_close_sample_key, temp_close_sample, cost_to_close_sample)

            else:
                # find lowest cost to connect both trees
                temp_path_cost = sample["cost_to_source"] + temp_close_sample["cost_to_source"] + self.distance(temp_close_sample["pos"], sample["pos"])

                if temp_path_cost < cheapest_path_cost:

                    cheapest_path_cost = temp_path_cost
                    cheapest_path_cost_sample_key = temp_close_sample_key 

        # found a path to connect both trees
        if cheapest_path_cost_sample_key is not None:
            self.shortest_paths[cheapest_path_cost] = {"sample1_key": sample_key, "sample2_key": cheapest_path_cost_sample_key}

        # TODO: situation which should remove the shortest path thingy


    def add_sample(self, sample: list, prev_key: int, cost_to_source) -> int:
        """ adds sample to all existing samples. """

        # check if the x and y positions already exist
        assert not self.x_sorted.__contains__(sample[0]), f"x position {sample[0]} already exist in sorted x positions"
        assert not self.y_sorted.__contains__(sample[1]), f"y position {sample[1]} already exist in sorted y positions"

        key = self.create_unique_id()

        self.samples[prev_key]["next_sample_keys"].append(key)

        self.samples[key] = {
                "pos": [sample[0], sample[1]],
                "cost_to_source": cost_to_source,
                "prev_sample_key": prev_key,
                "next_sample_keys": [],
                "in_tree": self.samples[prev_key]["in_tree"]}

        self.x_sorted[sample[0]] = key
        self.y_sorted[sample[1]]= key

        self.n_samples += 1

        return key

    def project_to_connectivity_graph(self, sample: list, project_to_sample_key: int) -> list:
        """ projects the sample closer to the closest existing sample in the connectivity graphs. """

        # TODO: using vectors speeds up, convert sin/cos calculation to vector calculation
        project_to_sample = self.samples[project_to_sample_key]

        if self.distance(project_to_sample["pos"], sample) > self.step_size:

            theta = np.arctan2(sample[1]-project_to_sample["pos"][1], sample[0]-project_to_sample["pos"][0])
            x_new = project_to_sample["pos"][0] + np.cos(theta) * self.step_size
            y_new = project_to_sample["pos"][1] + np.sin(theta) * self.step_size

            return [x_new, y_new]
        else:
            return sample

    def update_cost_sample(self, sample_key: int, sample: dict, new_cost: float):
        """ update sample cost for sample and next samples, including cost for shortest paths. """

        # update cost for this sample, 

        for next_sample_key in sample["next_sample_keys"]:
            
            
            next_sample = self.samples[next_sample_key]
            next_sample_new_cost = new_cost + self.distance(next_sample["pos"], sample["pos"])

            if next_sample["in_tree"] == sample["in_tree"]:

                # recursively update new costs
                self.update_cost_sample(next_sample_key, next_sample, next_sample_new_cost)

            else:

                # update the existing shortest path found shortest path
                shortest_path_key = sample["cost_to_source"] + self.distance(sample["pos"], next_sample["pos"]) + next_sample["cost_to_source"]
                self.shortest_paths.pop(shortest_path_key)

                new_shortest_path_key = next_sample_new_cost + next_sample["cost_to_source"]
                self.shortest_paths[new_shortest_path_key] = {"sample1_key": next_sample_key, "sample2_key": sample_key}


        # if the new samples deletes a shortest path?





    def distance(self, sample1: list, sample2: list) -> float:
        return np.linalg.norm([sample1[0] - sample2[0], sample1[1] - sample2[1]])

    def extract_shortest_path(self) -> list:
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

        while source_sample["prev_sample_key"] != self.source_tree_key:
            reversed_path.append(source_sample["pos"])
            source_sample = self.samples[source_sample["prev_sample_key"]]

        reversed_path.append(self.samples[self.source_tree_key]["pos"])
        path = list(reversed(reversed_path))

        while target_sample["prev_sample_key"] != self.target_tree_key:
            path.append(target_sample["pos"])
            target_sample = self.samples[target_sample["prev_sample_key"]]

        path.append(self.samples[self.target_tree_key]["pos"])

        return path



