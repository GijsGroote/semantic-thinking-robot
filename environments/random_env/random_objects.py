import random
from typing import Tuple
import math
import numpy as np
from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.state import State
from robot_brain.obstacle import Obstacle, UNMOVABLE
from robot_brain.global_planning.hgraph.local_planning.graph_based.\
        circle_obstacle_path_estimator import CircleObstaclePathEstimator
from helper_functions.figures import get_random_color

class RandomObject():
    """ Generate random objects. """

    def __init__(self,
            grid_x_length: float,
            grid_y_length: float,
            min_dimension: float,
            max_dimension: float,
            max_weight: float):

        self.grid_x_length = grid_x_length
        self.grid_y_length = grid_y_length

        self.box_counter = 0
        self.cylinder_counter = 0
        self.movable_added = False

        self.min_dimension = min_dimension
        self.max_dimension = max_dimension
        self.max_weight = max_weight
        self.occupancy_graph = None


    # TODO: function creates obstacles and a task, the task can have target locations in which obstacles overlap
    # thus both cannot be at their respective target location at the same time.
    def create_random_objects_and_task(self, n_unmovable_obstacles: int, n_movable_obstacles: int, n_subtasks: int):
        """ create movable and unmovable obstacles and a task. """

        assert n_subtasks <= n_movable_obstacles, "number of subtasks cannot exceed the number of movable obstacles"

        obstacles = {}

        # create unmovable obstacles
        unmovable_obstacles = {}
        for _ in range(n_unmovable_obstacles):
            (random_box, position, orientation) = self._create_unmovable_random_box()
            obstacles[random_box.name()] = random_box
            temp_obst = Obstacle("name",
                    State(pos=np.array(position), ang_p=np.array([0, 0, orientation])), random_box)
            temp_obst.type = UNMOVABLE
            unmovable_obstacles[random_box.name()] = temp_obst

            (random_cylinder, position, orientation) = self._create_unmovable_random_cylinder()
            obstacles[random_cylinder.name()] = random_cylinder
            temp_obst = Obstacle("name",
                    State(pos=np.array(position), ang_p=np.array([0, 0, orientation])), random_cylinder)
            temp_obst.type = UNMOVABLE
            unmovable_obstacles[random_cylinder.name()] = temp_obst

        self.occupancy_graph = CircleObstaclePathEstimator(
                cell_size = 0.5,
                grid_x_length = self.grid_x_length,
                grid_y_length = self.grid_y_length,
                obstacles = unmovable_obstacles,
                obst_cart_2d = np.array([0,0]),
                obst_name = "no_obst_name",
                obst_radius = self.max_dimension/2)

        # create movable obstacles
        movable_obstacles = {}
        for _ in range(n_movable_obstacles):
            (random_box, position, orientation) = self._create_movable_random_box()
            movable_obstacles[random_box.name()] = random_box
            obstacles[random_box.name()] = random_box
            (random_cylinder, position, orientation) = self._create_movable_random_cylinder()
            movable_obstacles[random_cylinder.name()] = random_cylinder
            obstacles[random_cylinder.name()] = random_cylinder

        # create task
        task = []
        for _ in range(n_subtasks):
            temp_key = random.choice(list(movable_obstacles))
            p2d = self._get_random_2d_pose()
            task.append((movable_obstacles[temp_key].name(), State(pos=np.array([p2d[0],
                p2d[1], 0]), ang_p=np.array([0,0,p2d[2]]))))
            movable_obstacles.pop(temp_key)

        return (obstacles, task)

    def _create_movable_random_box(self) -> Tuple[BoxObstacle, list, float]:
        """ return movable box with random propeties. """
        self.movable_added = True
        return self._create_random_box(True, random.random()*self.max_weight)

    def _create_unmovable_random_box(self) -> Tuple[BoxObstacle, list, float]:
        """ return movable box with random propeties. """
        assert not self.movable_added, "first create unmovable obstacle, then create unmovable obstacles."
        return self._create_random_box(False, random.random()*self.max_weight)

    def _create_movable_random_cylinder(self) -> Tuple[CylinderObstacle, list, float]:
        """ return movable cylinder with random propeties. """
        self.movable_added = True
        return self._create_random_cylinder(True, random.random()*self.max_weight)

    def _create_unmovable_random_cylinder(self) -> Tuple[CylinderObstacle, list, float]:
        """ return movable cylinder with random propeties. """
        assert not self.movable_added, "first create unmovable obstacle, then create unmovable obstacles."
        return self._create_random_cylinder(False, random.random()*self.max_weight)

    def _create_random_box(self, movable: bool, mass: float) -> Tuple[BoxObstacle, list, float]:
        """ returns a box with random dimensions, location, color and weight. """

        orientation = random.random()*2*math.pi
        length = random.uniform(self.min_dimension, self.max_dimension)
        width = random.uniform(self.min_dimension, self.max_dimension)
        height = random.uniform(self.min_dimension, min(length, width))
        if height > 0.5*min(length, width):
            height = 0.5*min(length, width)
        position = self._get_random_position(height)

        box_dict = {
            "movable": movable,
            "mass": mass,
            "orientation": [0, 0, orientation],
            "type": "box",
            "color": get_random_color(),
            "position": position,
            "geometry": {
                "length": length,
                "width": width,
                "height": height},
        }

        self.box_counter += 1

        return (BoxObstacle(name="rand_box_"+str(self.box_counter), content_dict=box_dict), position, orientation)

    def _create_random_cylinder(self, movable: bool, mass: float) -> Tuple[CylinderObstacle, list, float]:
        """ returns a cylinder with random dimensions, location, color and weight. """
        orientation = random.random()*2*math.pi
        radius = random.uniform(self.min_dimension, 0.5*self.max_dimension)
        height = random.uniform(self.min_dimension, 0.5*self.max_dimension)
        if height > 0.25*radius:
            height = 0.25*radius
        position = self._get_random_position(height)

        cylinder_dict = {
            "movable": movable,
            "mass": mass,
            "orientation": [0, 0, orientation],
            "type": "cylinder",
            "color": get_random_color(),
            "position": position,
            "geometry": {
                "radius": radius,
                "height": height},
        }

        self.cylinder_counter += 1
        return (CylinderObstacle(name="rand_cylinder_"+str(self.cylinder_counter),
            content_dict=cylinder_dict), position, orientation)

    def _get_random_2d_pose(self) -> np.ndarray:
        """ return a random 2d pose that lies in free space. """
        random_2d_pose = np.array([random.uniform(-1,1)*(self.grid_x_length-self.max_dimension)/2,
               random.uniform(-1,1)*(self.grid_y_length-self.max_dimension)/2,
               random.uniform(0, 2*math.pi)])

        if self.occupancy_graph.occupancy(random_2d_pose[0:2]) == 0:
            return random_2d_pose
        else:
            return self._get_random_2d_pose()

    def _get_random_position(self, height):
        """ return a random position. """
        return [random.uniform(-1,1)*(self.grid_x_length-self.max_dimension)/2,
               random.uniform(-1,1)*(self.grid_y_length-self.max_dimension)/2,
               height/2]
