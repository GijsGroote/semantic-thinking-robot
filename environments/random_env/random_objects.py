import random
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
        self.movable_obst_dicts = {}
        self.unmovable_obst_dicts = {}
        self.movable_added = False

        self.min_dimension = min_dimension
        self.max_dimension = max_dimension
        self.max_weight = max_weight
        self.occupancy_graph = None

        self.n_unmovable_obstacles = None
        self.n_movable_obstacles = None
        self.n_subtasks = None

    def create_random_objects(self,
            n_unmovable_obstacles: int,
            n_movable_obstacles: int,
            n_subtasks: int):
        """ create movable and unmovable obstacles. """

        assert n_subtasks <= n_movable_obstacles, "number of subtasks cannot exceed the number of movable obstacles"
        self.n_unmovable_obstacles = n_unmovable_obstacles
        self.n_movable_obstacles = n_movable_obstacles
        self.n_subtasks = n_subtasks

        # create unmovable obstacle info
        for _ in range(n_unmovable_obstacles):

            if random.choice([True, False]):
                self._create_unmovable_random_box()
            else:
                self._create_unmovable_random_cylinder()

         # create movable obstacles
        for _ in range(n_movable_obstacles):

            if random.choice([True, False]):
                self._create_movable_random_box()

            else:
                self._create_movable_random_cylinder()

    def reshuffle_env(self) -> dict:
        """ reshuffles the states of every obstacle. """

        # reshuffle unmovable obstacles
        for obst_dict in self.unmovable_obst_dicts.values():

            if obst_dict["type"] == "box":
                p2d = self._get_random_2d_pose()
                obst_dict["position"] = [p2d[0], p2d[1],
                        0.5*obst_dict["geometry"]["height"]]
                obst_dict["orientation"] = [0, 0, p2d[2]]

            elif obst_dict["type"] == "cylinder":
                xy = self._get_random_xy_position()
                obst_dict["position"] = [xy[0], xy[1],
                        0.5*obst_dict["geometry"]["height"]]
                obst_dict["orientation"] = [0, 0, 0]
            else:
                raise ValueError(f"unknown type: {obst_dict['type']}")

        self._create_occupancy_graph()

        # reshuffle movable obstacles
        for obst_dict in self.movable_obst_dicts.values():

            if obst_dict["type"] == "box":
                p2d = self._get_random_2d_pose_in_free_space()
                obst_dict["position"] = [p2d[0], p2d[1],
                        0.5*obst_dict["geometry"]["height"]]
                obst_dict["orientation"] = [0, 0, p2d[2]]

            elif obst_dict["type"] == "cylinder":
                xy = self._get_random_xy_position_in_free_space()
                obst_dict["position"] = [xy[0], xy[1],
                        0.5*obst_dict["geometry"]["height"]]
                obst_dict["orientation"] = [0, 0, 0]
            else:
                raise ValueError(f"unknown type: {obst_dict['type']}")

        # for (obst_key, obst) in self.movable_obst_dicts.items():
        #
        #
        #     if isinstance(obst, CylinderObstacle):
        #         rand_xy_position = self._get_random_position(2*obst.state.pos[2])
        #         temp_state = State(pos=np.array([rand_xy_position[0], rand_xy_position[1], rand_xy_position[2]]))
        #
        #     elif isinstance(obst, CylinderObstacle):
        #         rand_2d_pose = self._get_random_pose()
        #         temp_state = State(pos=np.array([rand_2d_pose[0], rand_2d_pose[1], rand_xy_position[2]]),
        #             ang_p=np.array([0,0,rand_2d_pose[2]]))
        #     else:
        #         raise ValueError(f"unknown obstacle encoutered, {type(obst)}")
        #
        return self._convert_to_obstacles()

    # TODO: function creates obstacles and a task, the task can have target locations in which obstacles overlap
    # thus both cannot be at their respective target location at the same time.
    def create_task(self, obstacles) -> list:
        """ creates a task for the obstacles in the environment. """

        task = []
        movable_obst_dicts_copy = self.movable_obst_dicts.copy()
        for _ in range(self.n_subtasks):
            temp_key = random.choice(list(movable_obst_dicts_copy))
            target_state = self._get_random_state_in_free_space(self.movable_obst_dicts[temp_key]["position"][2])

            task.append((obstacles[temp_key].name(), target_state))
            movable_obst_dicts_copy.pop(temp_key)

        return task

    def _create_occupancy_graph(self):
        """ create an occupancy graph with the unmovable obstacles. """

        unmovable_obstacles = {}
        for (key, obst_dict) in self.unmovable_obst_dicts.items():

            if obst_dict["type"] == "box":
                obst = BoxObstacle(name="inside_setup_random_env", content_dict=obst_dict)

            elif obst_dict["type"] == "cylinder":
                obst = CylinderObstacle(name="inside_setup_random_env", content_dict=obst_dict)
            else:
                raise ValueError(f"unknown type: {obst_dict['type']}")

            temp_obst = Obstacle("inside_setup_random_env",
                State(pos=np.array(obst_dict["position"]),
                    ang_p=np.array(obst_dict["orientation"])),
                obst)
            temp_obst.type = UNMOVABLE

            unmovable_obstacles[key] = temp_obst

        self.occupancy_graph = CircleObstaclePathEstimator(
                cell_size = 0.5,
                grid_x_length = self.grid_x_length,
                grid_y_length = self.grid_y_length,
                obstacles = unmovable_obstacles,
                obst_cart_2d = np.array([0,0]),
                obst_name = "no_obst_name",
                obst_radius = self.max_dimension/2)

        print(f'now the occu graph is created see {self.occupancy_graph}')

    def _convert_to_obstacles(self) -> dict:
        """ convert obstacle information to obstacles. """

        obstacles = {}

        # unmovable obstacles
        for (key, obst_dict) in self.unmovable_obst_dicts.items():

            if obst_dict["type"] == "box":
                obstacles[key] = BoxObstacle(name=key, content_dict=obst_dict)
            elif obst_dict["type"] == "cylinder":
                obstacles[key] = CylinderObstacle(name=key, content_dict=obst_dict)
            else:
                raise ValueError(f"unknown type: {obst_dict['type']}")

        # movable obstacles
        for (key, obst_dict) in self.movable_obst_dicts.items():
            if obst_dict["type"] == "box":
                obstacles[key] = BoxObstacle(name=key, content_dict=obst_dict)
            elif obst_dict["type"] == "cylinder":
                obstacles[key] = CylinderObstacle(name=key, content_dict=obst_dict)
            else:
                raise ValueError(f"unknown type: {obst_dict['type']}")

        return obstacles


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
        height = random.uniform(self.min_dimension, 0.5*self.max_dimension)

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

    def _get_random_2d_pose(self) -> list:
        """ return a random 2d pose in boundaries. """
        return [random.uniform(-1,1)*(self.grid_x_length-self.max_dimension)/2,
               random.uniform(-1,1)*(self.grid_y_length-self.max_dimension)/2,
               random.random()*2*math.pi]

    def _get_random_2d_pose_in_free_space(self) -> list:
        """ return a random 2d pose that lies in free space. """
        random_2d_pose = [random.uniform(-1,1)*(self.grid_x_length-self.max_dimension)/2,
               random.uniform(-1,1)*(self.grid_y_length-self.max_dimension)/2,
               random.uniform(0, 2*math.pi)]

        if self.occupancy_graph.occupancy(np.array(random_2d_pose[0:2])) == 0:
            return random_2d_pose
        else:
            return self._get_random_2d_pose()

    def _get_random_xy_position(self) -> list:
        """ return a random position in boundaries. """
        return [random.uniform(-1,1)*(self.grid_x_length-self.max_dimension)/2,
               random.uniform(-1,1)*(self.grid_y_length-self.max_dimension)/2]

    def _get_random_xy_position_in_free_space(self) -> list:
        """ return a rondom xy position that lies in free space. """
        random_2d_pose = [random.uniform(-1,1)*(self.grid_x_length-self.max_dimension)/2,
               random.uniform(-1,1)*(self.grid_y_length-self.max_dimension)/2]

        if self.occupancy_graph.occupancy(np.array(random_2d_pose)) == 0:
            return random_2d_pose
        else:
            return self._get_random_xy_position_in_free_space()
