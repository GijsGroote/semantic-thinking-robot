
import numpy as np
from robot_brain.obstacle import Obstacle
from robot_brain.global_planning.hgraph.local_planning.graph_based.rectangular_robot_configuration_grid_map import RectangularRobotConfigurationGridMap
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_robot_configuration_grid_map import CircleRobotConfigurationGridMap 
from robot_brain.global_planning.hgraph.local_planning.sample_based.motion_planner import MotionPlanner
from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle 


class DriveMotionPlanner(MotionPlanner):
    """ Motion planner, using a double rapid randomly tree star (RRT*) to search a path for the robot to track. """

    def __init__(self, 
        grid_x_length: float,
        grid_y_length: float,
        obstacles: dict,
        obstacle: Obstacle):

        MotionPlanner.__init__(self, grid_x_length, grid_y_length, obstacle)

        if isinstance(robot.properties, CylinderObstacle):
            self.configuration_space_grid_map = CircleRobotConfigurationGridMap(
                cell_size=0.2,
                grid_x_length=grid_x_length,
                grid_y_length=grid_y_length,
                obstacles=obstacles,
                robot_cart_2d=robot.state.get_xy_position(),
                robot_radius=robot.properties.radius()) # radius or radius()???

        elif isinstance(robot.properties, BoxObstacle):
            self.configuration_space_grid_map = RectangularRobotConfigurationGridMap(
                cell_size=0.2,
                grid_x_length=grid_x_length,
                grid_y_length=grid_y_length,
                obstacles=obstacles,
                robot_cart_2d=robot.state.get_xy_position(),
                n_orientations= 36,
                robot_x_length=robot.properties.length,
                robot_y_length=robot.properties.width)
        else:
            raise ValueError("The robot has an unknown obstacle")



