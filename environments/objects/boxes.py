from motion_planning_env.box_obstacle import BoxObstacle
import math

box_dict = {
    "movable": True,
    "orientation": [0, 0, 0],
    "mass": 3,
    "type": "box",
    "color": [0/255, 255/255, 0/255, 3],
    "position": [3.0, 0.0, 0.1],
    "geometry": {"length": 2, "width": 2, "height": 0.1},
}

box = BoxObstacle(name="simple_box", content_dict=box_dict)
