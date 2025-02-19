import os
import math
from motion_planning_env.urdf_obstacle import UrdfObstacle
from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle

# blockade
blockade_obstacles = {
        "simpleBox": BoxObstacle(name="simpleBox", content_dict={
            "movable": True,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [22, 63, 88],
            "position": [1, 2, 0.7],
            "geometry": {"length": 1.4, "width": 1.4, "height": 1.4},
            }),
        "simpleCylinder": CylinderObstacle(name="simpleCylinder", content_dict={
            "movable": True,
            "mass": 3,
            "color": [13, 180, 185],
            "position": [1.5, 0, 0.15],
            "type": "cylinder",
            "geometry": {"radius": 0.5, "height": 0.3},
            }),
        "wall1": BoxObstacle(name="wall1", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, math.pi/2],
            "color": [245, 181, 27, 1],
            "position": [4.1, 0, 0.2],
            "geometry": {"length": 2.7, "width": 0.3, "height": 0.4},
            }),
        "wall2": BoxObstacle(name="wall2", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,0],
            "color": [245, 181, 27, 1],
            "position":[3, 1.2, 0.2],
            "geometry": {"length": 2, "width": 0.3, "height": 0.4},
            }),
        "wall3": BoxObstacle(name="wall3", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,0],
            "color": [245, 181, 27, 1],
            "position": [3, -1.2, 0.2],
            "geometry": {"length": 2, "width": 0.3, "height": 0.4},
            },
            ),
        }

# swap
box_dict = {
    "color": [44, 95, 45],
    "position": [2, 1, 0.3],
    "type": "box",
    "geometry": {"length": 0.6, "width": 0.5, "height": 0.6},
}

cylinder_dict = {
    "color": [155, 188, 98],
    "position": [2, -1, 0.15],
    "type": "cylinder",
    "geometry": {"radius": 0.4, "height": 0.3},
}

duck_small_dict= {
    "position": [2, -1, 0.25],
    "orientation": [math.pi/2, 0, 0],
    "type": "urdf",
    "geometry": {
        "urdf": os.path.join(os.path.dirname(__file__), "./obstacle_data/duck/duck.urdf"),
    }
}

swap = {"small_box": BoxObstacle(name="small_box", content_dict=box_dict),
        "small_cylinder": CylinderObstacle(name="small_cylinder", content_dict=cylinder_dict),
        "small_duck": UrdfObstacle(name="small_duck", content_dict=duck_small_dict)}
