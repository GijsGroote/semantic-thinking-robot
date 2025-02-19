"""
RandomObject class will create random objects
between the specified upper and lower bounds.
The objects can be reshuffled to give them a
new position in the robot environment, for
generated/reshuffled tasks, configuration space
is used to make sure that the task is feasible.
"""
import random
import math
import numpy as np
from motion_planning_env.free_collision_obstacle import FreeCollisionObstacle
from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle


from robot_brain.global_variables import POINT_ROBOT_RADIUS
from robot_brain.state import State
from robot_brain.object import Object, FREE, MOVABLE, UNKNOWN, UNMOVABLE
from robot_brain.local_planning.graph_based.circle_object_path_estimator import CircleObjectPathEstimator
from robot_brain.local_planning.graph_based.rectangle_object_path_estimator import RectangleObjectPathEstimator

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

        # hold obstacle information, but do not yet create the Obstacles
        self.box_counter = 0
        self.cylinder_counter = 0
        self.movable_obst_dicts = {}    # movable obstacle data (content_dict, thus not a FreeCollisionObstacle)
        self.unmovable_obst_dicts = {}  # unmovable obstacle data
        self.n_movable_objects = None

        # created obstacles a position
        self.init_objects = {}
        self.target_objects =  {}
        self.movable_added = False
        self.unmovable_objects = {}
        self.subtask_objects = {}
        self._clear_obstacle_dicts()

        self.min_dimension = min_dimension
        self.max_dimension = max_dimension
        self.max_weight = max_weight
        self.occupancy_graph = None

        self.n_subtasks = None

    def create_random_objects(self,
            n_unmovable_objects: int,
            n_movable_objects: int):
        """ create movable and unmovable objects randomly. """

        self.n_movable_objects = n_movable_objects

        # create unmovable obstacle info dict
        for _ in range(n_unmovable_objects):
            if random.choice([True, False]):
                self._create_unmovable_random_box()
            else:
                self._create_unmovable_random_cylinder()

         # create movable objects info dict
        for _ in range(n_movable_objects):
            if random.choice([True, False]):
                self._create_movable_random_box()
            else:
                self._create_movable_random_cylinder()

    def reshuffle_env(self) -> dict:
        """ reshuffles initial states for all objects. """

        self._clear_obstacle_dicts()
        objects = {}

        # reshuffle unmovable objects
        for (obst_key, obst_dict) in self.unmovable_obst_dicts.items():

            if obst_dict["type"] == "box":
                p2d = self._get_random_2d_pose()
                obst_dict["position"] = [p2d[0], p2d[1],
                        0.5*obst_dict["geometry"]["height"]]
                obst_dict["orientation"] = [0, 0, p2d[2]]
                obst = BoxObstacle(name=obst_key, content_dict=obst_dict)
                objects[obst_key] = obst

            elif obst_dict["type"] == "cylinder":
                xy_pos = self._get_random_xy_position()
                obst_dict["position"] = [xy_pos[0], xy_pos[1],
                        0.5*obst_dict["geometry"]["height"]]
                obst_dict["orientation"] = [0, 0, 0]
                obst = CylinderObstacle(name=obst_key, content_dict=obst_dict)
                objects[obst_key] = obst

            else:
                raise ValueError(f"unknown type: {obst_dict['type']}")

            # keep track of all unmovable objects
            temp_obst = Object("inside_setup_random_env",
                State(pos=np.array(obst_dict["position"]),
                    ang_p=np.array(obst_dict["orientation"])),
                obst)
            temp_obst.type = UNMOVABLE
            self.unmovable_objects[obst_key] = temp_obst

        # reshuffle movable objects
        for (obst_key, obst_dict) in self.movable_obst_dicts.items():
            obst = self._create_movable_init_obstacle(obst_key, obst_dict)
            objects[obst_key] = obst

            # keep track of all movable objects
            temp_obst = Object("inside_setup_random_env",
                    State(pos=np.array(obst_dict["position"]),
                        ang_p=np.array(obst_dict["orientation"])),
                    obst)
            temp_obst.type = UNMOVABLE
            self.init_objects[obst_key] = temp_obst

        return objects

    def create_drive_task(self, n_subtasks: int) -> list:
        """ creates a driving task for the robot in the environment. """

        task = []

        # create configuration space for robot
        occupancy_graph = CircleObjectPathEstimator(
            cell_size = 0.25,
            grid_x_length = self.grid_x_length,
            grid_y_length = self.grid_y_length,
            objects = self.unmovable_objects,
            obst_cart_2d = np.array([0,0]),
            obst_name = "must_be_in_RandomObject_class",
            obst_radius = POINT_ROBOT_RADIUS)


        pos = self._get_random_xy_position()

        for _ in range(n_subtasks):
            # search position in free space
            # NOTE: potentially in the while loop forever
            pos = self._get_random_xy_position()
            while occupancy_graph.occupancy(pos) != FREE:
                pos = self._get_random_xy_position()
            task.append(("robot", State(pos=np.array([pos[0], pos[1], 0]))))

        return task

    def create_push_task(self, n_subtasks) -> list:
        """ creates a task for the objects in the environment. """

        assert n_subtasks <= self.n_movable_objects, "number of subtasks cannot exceed the number of movable obstacles"


        self.n_subtasks = n_subtasks
        movable_obst_list = list(self.movable_obst_dicts.copy().items())

        if len(self.subtask_objects) == 0:
            for _ in range(n_subtasks):
                # NOTE: without removing anything, you can pick the same object here

                (obst_key, obst_dict) = random.choice(movable_obst_list)
                self.subtask_objects[obst_key] = obst_dict


        task = []
        for (obst_key, obst_dict) in self.subtask_objects.items():

            target_state = self.create_object_target_state(obst_key)

            temp_obst = Object("inside_setup_random_env",
                    target_state,
                    self.init_objects[obst_key].properties)
            temp_obst.type = UNMOVABLE
            # keep track of all target states
            self.target_objects[obst_key] = temp_obst

            # self.init_objects[obst_key] = Object("inside_setup_random_env",
            #     State(pos=np.array(obst_dict["position"]),
            #         ang_p=np.array(obst_dict["orientation"])),
            #     obst)
            # temp_obst.type = UNMOVABLE

            task.append((obst_key, target_state))
            movable_obst_list.remove((obst_key, obst_dict))


        return task

    def create_object_target_state(self, obj_key) -> State:
        """ return object target state. """
        obst_dict = self.movable_obst_dicts[obj_key]


        if obst_dict["type"] == "box":
            occupancy_graph = RectangleObjectPathEstimator(
                cell_size = 0.25,
                grid_x_length = self.grid_x_length,
                grid_y_length = self.grid_y_length,
                objects = {**self.unmovable_objects, **self.init_objects, **self.target_objects},
                obst_cart_2d = np.array([0,0]),
                obst_name = "must_be_in_RandomObject_class",
                obst_x_length = obst_dict["geometry"]["length"],
                obst_y_length = obst_dict["geometry"]["width"],
                n_orientations = 1,
                single_orientation = True,
                orientation = self.init_objects[obj_key].state.get_2d_pose()[2])

            pose_2d = self._get_random_2d_pose()
            # NOTE: potentially in the while loop forever
            while occupancy_graph.occupancy(pose_2d) != 0 or np.linalg.norm(self.init_objects[obj_key].state.get_xy_position()-pose_2d[0:2]) > 2:
                pose_2d = self._get_random_2d_pose()

            return State(pos=np.array([pose_2d[0], pose_2d[1], 0]), ang_p=np.array([0,0,pose_2d[2]]))

        elif obst_dict["type"] == "cylinder":
            occupancy_graph = CircleObjectPathEstimator(
                cell_size = 0.25,
                grid_x_length = self.grid_x_length,
                grid_y_length = self.grid_y_length,
                objects = {**self.unmovable_objects, **self.init_objects, **self.target_objects},
                obst_cart_2d = np.array([0,0]),
                obst_name = "must_be_in_RandomObject_class",
                obst_radius = obst_dict["geometry"]["radius"])

            pos = self._get_random_xy_position()
            # NOTE: potentially in the while loop forever
            while occupancy_graph.occupancy(pos) != 0 or np.linalg.norm(self.init_objects[obj_key].state.get_xy_position()-pos) > 2:
                pos = self._get_random_xy_position()


            return State(pos=pos)




    def _create_unmovable_objects(self):
        """ creates unmovable objects from unmovable obstacles dicts. """

        self.unmovable_objects = {}

        for (key, obst_dict) in self.unmovable_obst_dicts.items():

            if obst_dict["type"] == "box":
                obst = BoxObstacle(name="must_be_in_RandomObject_class", content_dict=obst_dict)

            elif obst_dict["type"] == "cylinder":
                obst = CylinderObstacle(name="must_be_in_RandomObject_class", content_dict=obst_dict)
            else:
                raise ValueError(f"unknown type: {obst_dict['type']}")

            temp_obst = Object("must_be_in_RandomObject_class",
                State(pos=np.array(obst_dict["position"]),
                    ang_p=np.array(obst_dict["orientation"])),
                obst)
            temp_obst.type = UNMOVABLE

            self.unmovable_objects[key] = temp_obst

    def _create_movable_init_obstacle(self, obst_key: str, obst_dict: dict) -> FreeCollisionObstacle:
        """ create movable obstacle in free space (thus also not in movable space). """
        if obst_dict["type"] == "box":
            return self._create_box_obstacle_free(obst_key, obst_dict, self.init_objects)

        elif obst_dict["type"] == "cylinder":
            return self._create_cylinder_obstacle_free(obst_key, obst_dict, self.init_objects)

        else:
            raise ValueError(f"unknown type: {obst_dict['type']}")

    def _create_movable_target_object(self, obst_key: str, obst_dict: dict) -> FreeCollisionObstacle:
        """ create movable obstacle in free space (thus also not in movable space). """
        if obst_dict["type"] == "box":
            return self._create_box_obstacle_free(obst_key, obst_dict, self.target_objects)

        elif obst_dict["type"] == "cylinder":
            return self._create_cylinder_obstacle_free(obst_key, obst_dict, self.target_objects)

        else:
            raise ValueError(f"unknown type: {obst_dict['type']}")

    def _create_box_obstacle_free(self, obst_key: str, obst_dict: dict, objects_dict: dict) -> BoxObstacle:
        """ create an occupancy graph with the unmovable and movable objects. """

        rand_orientation = random.random()*2*math.pi

        occupancy_graph = RectangleObjectPathEstimator(
            cell_size = 0.25,
            grid_x_length = self.grid_x_length,
            grid_y_length = self.grid_y_length,
            objects = {**self.unmovable_objects, **objects_dict},
            obst_cart_2d = np.array([0,0]),
            obst_name = "must_be_in_RandomObject_class",
            obst_x_length = obst_dict["geometry"]["length"],
            obst_y_length = obst_dict["geometry"]["width"],
            n_orientations = 1,
            single_orientation = True,
            orientation = rand_orientation)

        p2d = self._get_random_2d_pose()

        # NOTE: potentially in the while loop forever
        while occupancy_graph.occupancy(p2d) != 0:
            p2d = self._get_random_2d_pose()

        obst_dict["position"] = [p2d[0], p2d[1], 0.5*obst_dict["geometry"]["height"]]
        obst_dict["orientation"] = [0, 0, p2d[2]]
        return BoxObstacle(name=obst_key, content_dict=obst_dict)

    def _create_cylinder_obstacle_free(self, obst_key: str, obst_dict: dict, objects_dict: dict) -> CylinderObstacle:
        """ create an occupancy graph with the unmovable and movable objects. """

        occupancy_graph = CircleObjectPathEstimator(
            cell_size = 0.25,
            grid_x_length = self.grid_x_length,
            grid_y_length = self.grid_y_length,
            objects = {**self.unmovable_objects, **objects_dict},
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
        self._create_random_box(True, self._create_rand_weight())

    def _create_unmovable_random_box(self):
        """ return movable box with random propeties. """
        assert not self.movable_added, "first create unmovable obstacle, then create unmovable objects."
        self._create_random_box(False, self._create_rand_weight())
        # self._create_random_box(False, 0) # delete line above

    def _create_movable_random_cylinder(self):
        """ return movable cylinder with random propeties. """
        self.movable_added = True
        self._create_random_cylinder(True, self._create_rand_weight())

    def _create_unmovable_random_cylinder(self):
        """ return movable cylinder with random propeties. """
        assert not self.movable_added, "first create unmovable obstacle, then create unmovable objects."
        self._create_random_cylinder(False, self._create_rand_weight())
        # self._create_random_cylinder(False, 0) # delete line above

    def _create_rand_weight(self) -> float:
        """ return a random weight. """
        return max(1, random.random()*self.max_weight)

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
        self.init_objects = {}
        self.target_objects = {}
        self.movable_added = False
        self.unmovable_objects = {}

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
