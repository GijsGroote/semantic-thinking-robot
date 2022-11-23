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
                "in_tree": self.source_tree_key}

        self.samples[self.target_tree_key] = {
                "pos": [target_sample[0], target_sample[1]],
                "cost_to_source": 0,
                "prev_sample_key": self.target_tree_key,
                "in_tree": self.target_tree_key}

        self.n_samples = 2
        self.x_sorted = SortedDict({start_sample[0]: self.source_tree_key, target_sample[0]: self.target_tree_key})
        self.y_sorted = SortedDict({start_sample[1]: self.source_tree_key, target_sample[1]: self.target_tree_key})

    def search(self, start: State, target: State) -> list:
        """ search for a path between start and target state, raises error if no path can be found. """

        self.configuration_space_grid_map.update()
        # self.configuration_space_grid_map.shortest_path() # TODO: add samples to the RRT* planning algorithm 
        # before searching for a path

        self.setup(start.get_xy_position(), target.get_xy_position())

        # while keep_searching:
        while len(self.connect_trees) < 20:

            # generate random sample
            sample_rand = self.create_random_sample()
            # find closest sample
            sample_closest_key = self.get_closest_sample_key(sample_rand)
            
            # project it to existing samples
            sample_new = self.project_to_connectivity_graph(sample_rand, sample_closest_key)

            # find closeby samples

            # check subspace: obstacle-, movable- or unknown space
            space_id = self.configuration_space_grid_map.occupancy(np.array(sample_new))

            if space_id == 1: # obstacle space
                continue

            add_cost = 0

            if space_id == 2: # movable space
                # previous sample not in movable space -> add a high cost

                if self.configuration_space_grid_map.occupancy(np.array(self.samples[sample_closest_key]["pos"])) != 2:
                    print('someone is going into the movable space there')
                    add_cost = 15

            elif space_id == 3: # unkown space
                if self.configuration_space_grid_map.occupancy(np.array(self.samples[sample_closest_key]["pos"])) != 3:
                    add_cost = 25


            closest_total_cost = sys.float_info.max
            closest_key = None

            # TODO: if the search size is to small, there cannot be a parent sample found
            for key in self.get_closeby_sample_keys(sample_new, self.search_size/2):
                temp_sample = self.samples[key]
                temp_total_cost = temp_sample["cost_to_source"]+self.distance(temp_sample["pos"], sample_new)

                if temp_total_cost < closest_total_cost:
                    closest_total_cost = temp_total_cost
                    closest_key = key

            # TODO: temp fix make this better.
            if closest_key== None:
                continue

            # add new sample
            try: 
                new_sample_key = self.add_sample(sample_new, closest_key, closest_total_cost+add_cost)
            except AssertionError:
                warnings.warn("duplicate key found in sorted pose of sample")
                continue


            new_sample_in_tree = self.samples[closest_key]["in_tree"]

            # connect to the other tree
            closest_total_cost_other_tree = sys.float_info.max
            closest_key_other_tree = None
            for key in self.get_closeby_sample_keys(sample_new, self.search_size/2):
                temp_sample = self.samples[key]
                if temp_sample["in_tree"] != new_sample_in_tree:
                    temp_total_cost = temp_sample["cost_to_source"]+self.distance(temp_sample["pos"], sample_new)

                    if temp_total_cost < closest_total_cost_other_tree:
                        closest_total_cost_other_tree = temp_total_cost
                        closest_key_other_tree = key
            if closest_key_other_tree is not None:
                self.connect_trees[new_sample_key] = {"connected_to": closest_key_other_tree, "path_cost": closest_total_cost+closest_total_cost_other_tree}  
                
        # visualisation could be removed
        path = self.extract_shortest_path()
        self.visualise(save=False, shortest_path=path)

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

    def add_sample(self, sample: list, prev_key: int, cost_to_source) -> int:
        """ adds sample to all existing samples. """

        # check if the x and y positions already exist
        assert not self.x_sorted.__contains__(sample[0]), f"x position {sample[0]} already exist in sorted x positions"
        assert not self.y_sorted.__contains__(sample[1]), f"y position {sample[1]} already exist in sorted y positions"

        key = self.create_unique_id()

        self.samples[key] = {
                "pos": [sample[0], sample[1]],
                "cost_to_source": cost_to_source,
                "prev_sample_key": prev_key,
                # TODO: loc here below can become None (prev_key == None)
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

    def extract_shortest_path(self) -> list:
        """ Finds the shortest path after sampling. """
        if len(self.connect_trees) == 0:
            raise ValueError("start to target tree is not connected")

        lowest_key = sys.float_info.max
        lowest_path_cost = sys.float_info.max

        for key in self.connect_trees.keys():
            temp_cost = self.connect_trees[key]["path_cost"] 
            if temp_cost < lowest_path_cost:
                lowest_path_cost = temp_cost
                lowest_key = key

        print(f' the lowest cost is {lowest_path_cost}')

        sample1 = self.samples[lowest_key]
        sample2 = self.samples[self.connect_trees[lowest_key]["connected_to"]]
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



