import numpy as np    
import random
import sys
import time

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
        
        print("succesfully initialised DriveMotionPlanner")

    def search(self, start: State, target: State) -> list:
        """ search for a path between start and target state, raises error if no path can be found. """

        self.configuration_space_grid_map.update()
        # self.configuration_space_grid_map.shortest_path() # TODO: add samples to the RRT* planning algorithm 
        # before searching for a path

        self.connectivity_graph.setup(start.get_xy_position(), target.get_xy_position())

        # TESTING THE BIG O TIMES
        # times = []
        # iters = [1e1, 1e2, 1e3,1e4,1e5]#,1e6]
        # iters = [1e1, 1e2, 1e3]
            # starttime = time.time()
            # stoptime = time.time()
            # times.append(stoptime -starttime)
        # x and y given as array_like objects
        # fig = px.scatter(x=iters, y=times, log_x=True, log_y=True, marker_size=5)
        # fig.show() 
        # for n_iter in iters:

        keep_searching = True
        # while keep_searching:
        for i in range(130):

            # generate random sample
            sample_rand = self.create_random_sample()
            # find closest sample
            sample_closest_key = self.connectivity_graph.get_closest_sample_key(sample_rand)
            
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
                if self.configuration_space_grid_map.occupancy(np.array(self.connectivity_graph.samples[sample_closest_key])) != 2:
                    add_cost = 15

            elif space_id == 3: # unkown space
                if self.configuration_space_grid_map.occupancy(np.array(self.connectivity_graph.samples[sample_closest_key])) != 3:
                    add_cost = 25


            closest_key_cost = self.connectivity_graph.samples[sample_closest_key]["cost_to_source"]
            closest_total_cost = sys.float_info.max
            closest_key = None

            for key in self.connectivity_graph.get_closeby_sample_keys(sample_new, self.search_size/2):
                temp_sample = self.connectivity_graph.samples[key]
                temp_total_cost = temp_sample["cost_to_source"]+self.connectivity_graph.distance(temp_sample["pos"], sample_new)

                if temp_total_cost < closest_total_cost:
                    closest_total_cost = temp_total_cost
                    closest_key = key

            self.connectivity_graph.add_sample(sample_new, closest_key, closest_total_cost+add_cost)
        
        print("showing the connectivity graph")
        self.connectivity_graph.visualise(save=False)

        return []

    def create_random_sample(self) -> list:
        """ randomly generate sample inside grid boundaries. """
        x_rand = random.uniform(-self.grid_x_length/2, self.grid_x_length/2)
        y_rand = random.uniform(-self.grid_y_length/2, self.grid_y_length/2)

        return [x_rand, y_rand]

    def project_to_connectivity_graph(self, sample: list, project_to_sample_key: int) -> list:
        """ projects the sample closer to the closest existing sample in the connectivity graphs. """

        # TODO: using vectors speeds up, convert sin/cos calculation to vector calculation
        project_to_sample = self.connectivity_graph.samples[project_to_sample_key]

        if self.connectivity_graph.distance(project_to_sample["pos"], sample) > self.step_size:
            
            theta = np.arctan2(sample[1]-project_to_sample["pos"][1], sample[0]-project_to_sample["pos"][0])
            x_new = project_to_sample["pos"][0] + np.cos(theta) * self.step_size
            y_new = project_to_sample["pos"][1] + np.sin(theta) * self.step_size

            return [x_new, y_new]
        else:
            return sample


    def check_connecitvity(self, sample1: tuple, sample2: tuple) -> bool:
        """ check if 2 samples can be connected using a local planner. """
        raise NotImplementedError


