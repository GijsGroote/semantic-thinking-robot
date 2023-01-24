import random
import math
import numpy as np
from motion_planning_env.free_collision_obstacle import FreeCollisionObstacle
from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

from robot_brain.state import State
from robot_brain.obstacle import Obstacle, UNMOVABLE
from robot_brain.global_planning.hgraph.local_planning.graph_based.\
    circle_obstacle_path_estimator import CircleObstaclePathEstimator
from robot_brain.global_planning.hgraph.local_planning.graph_based.\
    rectangle_obstacle_path_estimator import RectangleObstaclePathEstimator

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
        self.movable_obst_dicts = {}
        self.unmovable_obst_dicts = {}

        self.init_obstacles = {}
        self.target_obstacles =  {}
        self.movable_added = False
        self.unmovable_obstacles = {}
        self._clear_obstacle_dicts()

        self.min_dimension = min_dimension
        self.max_dimension = max_dimension
        self.max_weight = max_weight
        self.occupancy_graph = None

        self.n_subtasks = None

    def create_random_obstacles(self,
            n_unmovable_obstacles: int,
            n_movable_obstacles: int,
            n_subtasks: int):
        """ create movable and unmovable obstacles randomly. """

        assert n_subtasks <= n_movable_obstacles, "number of subtasks cannot exceed the number of movable obstacles"
        self.n_subtasks = n_subtasks

        # create unmovable obstacle info dict
        for _ in range(n_unmovable_obstacles):
            if random.choice([True, False]):
                self._create_unmovable_random_box()
            else:
                self._create_unmovable_random_cylinder()

         # create movable obstacles info dict
        for _ in range(n_movable_obstacles):
            if random.choice([True, False]):
                self._create_movable_random_box()
            else:
                self._create_movable_random_cylinder()

    def reshuffle_env(self) -> dict:
        """ reshuffles initial states for all obstacles. """

        self._clear_obstacle_dicts()
        obstacles = {}

        # reshuffle unmovable obstacles
        for (obst_key, obst_dict) in self.unmovable_obst_dicts.items():

            if obst_dict["type"] == "box":
                p2d = self._get_random_2d_pose()
                obst_dict["position"] = [p2d[0], p2d[1],
                        0.5*obst_dict["geometry"]["height"]]
                obst_dict["orientation"] = [0, 0, p2d[2]]
                obst = BoxObstacle(name=obst_key, content_dict=obst_dict)
                obstacles[obst_key] = obst

            elif obst_dict["type"] == "cylinder":
                xy_pos = self._get_random_xy_position()
                obst_dict["position"] = [xy_pos[0], xy_pos[1],
                        0.5*obst_dict["geometry"]["height"]]
                obst_dict["orientation"] = [0, 0, 0]
                obst = CylinderObstacle(name=obst_key, content_dict=obst_dict)
                obstacles[obst_key] = obst

            else:
                raise ValueError(f"unknown type: {obst_dict['type']}")

            # keep track of all unmovable obstacles
            temp_obst = Obstacle("inside_setup_random_env",
                State(pos=np.array(obst_dict["position"]),
                    ang_p=np.array(obst_dict["orientation"])),
                obst)
            temp_obst.type = UNMOVABLE
            self.unmovable_obstacles[obst_key] = temp_obst

        # reshuffle movable obstacles
        for (obst_key, obst_dict) in self.movable_obst_dicts.items():
            obst = self._create_movable_init_obstacle(obst_key, obst_dict)
            obstacles[obst_key] = obst

            # keep track of all movable obstacles
            temp_obst = Obstacle("inside_setup_random_env",
                    State(pos=np.array(obst_dict["position"]),
                        ang_p=np.array(obst_dict["orientation"])),
                    obst)
            temp_obst.type = UNMOVABLE
            self.init_obstacles[obst_key] = temp_obst

        return obstacles

    def create_task(self) -> list:
        """ creates a task for the obstacles in the environment. """

        task = []
        movable_obst_list = list(self.movable_obst_dicts.copy().items())
        for _ in range(self.n_subtasks):

            (obst_key, obst_dict) = random.choice(movable_obst_list)

            obst = self._create_movable_target_state(obst_key, obst_dict)

            target_state = State(pos=np.array(obst_dict["position"]),
                        ang_p=np.array(obst_dict["orientation"]))

            temp_obst = Obstacle("inside_setup_random_env",
                    target_state,
                    obst)
            temp_obst.type = UNMOVABLE
            # keep track of all target states
            self.target_obstacles[obst_key] = temp_obst

            self.init_obstacles[obst_key] = Obstacle("inside_setup_random_env",
                State(pos=np.array(obst_dict["position"]),
                    ang_p=np.array(obst_dict["orientation"])),
                obst)
            temp_obst.type = UNMOVABLE

            task.append((obst_key, target_state))
            movable_obst_list.remove((obst_key, obst_dict))

        return task

    def _create_unmovable_obstacles(self):
        """ creates unmovable obstacles from unmovable obstacles dicts. """

        self.unmovable_obstacles = {}

        for (key, obst_dict) in self.unmovable_obst_dicts.items():

            if obst_dict["type"] == "box":
                obst = BoxObstacle(name="must_be_in_RandomObject_class", content_dict=obst_dict)

            elif obst_dict["type"] == "cylinder":
                obst = CylinderObstacle(name="must_be_in_RandomObject_class", content_dict=obst_dict)
            else:
                raise ValueError(f"unknown type: {obst_dict['type']}")

            temp_obst = Obstacle("must_be_in_RandomObject_class",
                State(pos=np.array(obst_dict["position"]),
                    ang_p=np.array(obst_dict["orientation"])),
                obst)
            temp_obst.type = UNMOVABLE

            self.unmovable_obstacles[key] = temp_obst

    def _create_movable_init_obstacle(self, obst_key: str, obst_dict: dict) -> FreeCollisionObstacle:
        """ create movable obstacle in free space (thus also not in movable space). """
        if obst_dict["type"] == "box":
            return self._create_box_obstacle_free(obst_key, obst_dict, self.init_obstacles)

        elif obst_dict["type"] == "cylinder":
            return self._create_cylinder_obstacle_free(obst_key, obst_dict, self.init_obstacles)

        else:
            raise ValueError(f"unknown type: {obst_dict['type']}")

    def _create_movable_target_state(self, obst_key: str, obst_dict: dict) -> FreeCollisionObstacle:
        """ create movable obstacle in free space (thus also not in movable space). """
        if obst_dict["type"] == "box":
            return self._create_box_obstacle_free(obst_key, obst_dict, self.target_obstacles)

        elif obst_dict["type"] == "cylinder":
            return self._create_cylinder_obstacle_free(obst_key, obst_dict, self.target_obstacles)

        else:
            raise ValueError(f"unknown type: {obst_dict['type']}")

    def _create_box_obstacle_free(self, obst_key: str, obst_dict: dict, obstacles_dict: dict) -> BoxObstacle:
        """ create an occupancy graph with the unmovable and movable obstacles. """

        rand_orientation = random.random()*2*math.pi

        occupancy_graph = RectangleObstaclePathEstimator(
            cell_size = 0.25,
            grid_x_length = self.grid_x_length,
            grid_y_length = self.grid_y_length,
            obstacles = {**self.unmovable_obstacles, **obstacles_dict},
            obst_cart_2d = np.array([0,0]),
            obst_name = "must_be_in_RandomObject_class",
            obst_x_length = obst_dict["geometry"]["length"],
            obst_y_length = obst_dict["geometry"]["width"],
            n_orientations = 1,
            single_orientation = True,
            orientation = rand_orientation)

        xy_pos = self._get_random_xy_position()
        p2d = [xy_pos[0], xy_pos[1], rand_orientation]

        # NOTE: potentially in the while loop forever
        while occupancy_graph.occupancy(p2d) != 0:
            p2d = self._get_random_2d_pose()

        obst_dict["position"] = [p2d[0], p2d[1], 0.5*obst_dict["geometry"]["height"]]
        obst_dict["orientation"] = [0, 0, p2d[2]]
        return BoxObstacle(name=obst_key, content_dict=obst_dict)

    def _create_cylinder_obstacle_free(self, obst_key: str, obst_dict: dict, obstacles_dict: dict) -> CylinderObstacle:
        """ create an occupancy graph with the unmovable and movable obstacles. """

        occupancy_graph = CircleObstaclePathEstimator(
            cell_size = 0.25,
            grid_x_length = self.grid_x_length,
            grid_y_length = self.grid_y_length,
            obstacles = {**self.unmovable_obstacles, **obstacles_dict},
            obst_cart_2d = np.array([0,0]),
            obst_name = "must_be_in_RandomObject_class",
            obst_radius = obst_dict["geometry"]["radius"])

        pos = self._get_random_xy_position()
        # NOTE: potentially in the while loop forever
        while occupancy_graph.occupancy(pos) != 0:
            pos = self._get_random_xy_position()

        obst_dict["position"] = [pos[0], pos[1], 0.5*obst_dict["geometry"]["height"]]
        obst_dict["orientation"] = [0, 0, 0]

        return CylinderObstacle(name=obst_key, content_dict=obst_dict)

    def _create_movable_random_box(self):
        """ return movable box with random propeties. """
        self.movable_added = True
        self._create_random_box(True, random.random()*self.max_weight)

    def _create_unmovable_random_box(self):
        """ return movable box with random propeties. """
        assert not self.movable_added, "first create unmovable obstacle, then create unmovable obstacles."
        self._create_random_box(False, random.random()*self.max_weight)

    def _create_movable_random_cylinder(self):
        """ return movable cylinder with random propeties. """
        self.movable_added = True
        self._create_random_cylinder(True, random.random()*self.max_weight)

    def _create_unmovable_random_cylinder(self):
        """ return movable cylinder with random propeties. """
        assert not self.movable_added, "first create unmovable obstacle, then create unmovable obstacles."
        self._create_random_cylinder(False, random.random()*self.max_weight)

    def _create_random_box(self, movable: bool, mass: float):
        """ returns a box with random dimensions, location, color and weight. """

        length = random.uniform(self.min_dimension, self.max_dimension)
        width = random.uniform(self.min_dimension, self.max_dimension)
        height = random.uniform(self.min_dimension, min(length, width))

        box_dict = {
            "movable": movable,
            "mass": mass,
            "type": "box",
            "color": get_random_color(),
            "geometry": {
                "length": length,
                "width": width,
                "height": height},
        }

        if movable:
            self.movable_obst_dicts["box_"+str(self.box_counter)] = box_dict
        else:
            self.unmovable_obst_dicts["box_"+str(self.box_counter)] = box_dict

        self.box_counter += 1

    def _create_random_cylinder(self, movable: bool, mass: float):
        """ returns a cylinder with random dimensions, location, color and weight. """
        radius = random.uniform(self.min_dimension, 0.5*self.max_dimension)
        height = random.uniform(self.min_dimension, radius)

        cylinder_dict = {
            "movable": movable,
            "mass": mass,
            "type": "cylinder",
            "color": get_random_color(),
            "geometry": {
                "radius": radius,
                "height": height},
        }

        if movable:
            self.movable_obst_dicts["cylinder_"+str(self.cylinder_counter)] = cylinder_dict
        else:
            self.unmovable_obst_dicts["cylinder_"+str(self.cylinder_counter)] = cylinder_dict

        self.cylinder_counter += 1

    def _clear_obstacle_dicts(self):
        """ clears the dictionaries. """
        self.init_obstacles = {}
        self.target_obstacles = {}
        self.movable_added = False
        self.unmovable_obstacles = {}

    def _get_random_2d_pose(self) -> list:
        """ return a random 2d pose in boundaries, exluding close to origin. """
        rand_pose = [random.uniform(-1,1)*(self.grid_x_length-self.max_dimension)/2,
               random.uniform(-1,1)*(self.grid_y_length-self.max_dimension)/2,
               random.random()*2*math.pi]
        if self._close_to_origin(rand_pose):
            return self._get_random_2d_pose()
        else:
            return rand_pose

    def _get_random_xy_position(self) -> list:
        """ return a random position in boundaries, excluding close to origin. """
        rand_xy_pos = [random.uniform(-1,1)*(self.grid_x_length-self.max_dimension)/2,
               random.uniform(-1,1)*(self.grid_y_length-self.max_dimension)/2]

        if self._close_to_origin(rand_xy_pos):
            return self._get_random_xy_position()
        else:
            return rand_xy_pos

    def _close_to_origin(self, xy_pos: list) -> bool:
        """ return true if the xy_pos is close to (0,0). """
        return np.linalg.norm([xy_pos[0], xy_pos[1]]) <= 0.5+self.max_dimension/2
