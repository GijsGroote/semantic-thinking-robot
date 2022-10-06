import os
import math
from motion_planning_env.urdf_obstacle import UrdfObstacle
from motion_planning_env.box_obstacle import BoxObstacle

# blockade
pushable_cube_dict = {
    "movable": True,
    "type": "box",
    "orientation": [0, 0, 0.2],
    "mass": 3,
    "color": [22, 63, 88],
    "position": [1, 2, 0.8],
    "geometry": {"length": 0.8, "width": 0.8, "height": 0.8},
}

urdf_duck_dict= {
    "position": [2, 0, 0.25],
    "orientation": [math.pi/2, 0, 0],
    "type": "urdf",
    "geometry": {
        "urdf": os.path.join(os.path.dirname(__file__), "./obstacle_data/duck/duck.urdf"),
    }
}

blockade_obstacles = {
        "simpleBox": BoxObstacle(name="simpleBox", content_dict=pushable_cube_dict),
        "urdf_duck": UrdfObstacle(name="urdf_duck", content_dict=urdf_duck_dict),
        "wall1": BoxObstacle(name="wall1", content_dict={
                "movable": False,
                "type": "box",
                "orientation": [0, 0, math.pi/2],
                "color": [245, 181, 27, 1],
                "position": [3.9, 0, 0.2],
                "geometry": {"length": 1.8, "width": 0.2, "height": 0.4},
            }),
        "wall2": BoxObstacle(name="wall2", content_dict={
                "movable": False,
                "type": "box",
                "orientation": [0,0,0],
                "color": [245, 181, 27, 1],
                "position":[3, 1, 0.2],
                "geometry": {"length": 2, "width": 0.2, "height": 0.4},
            }),
        "wall3": BoxObstacle(name="wall3", content_dict={
                "movable": False,
                "type": "box",
                "orientation": [0,0,0],
                "color": [245, 181, 27, 1],
                "position": [3, -1, 0.2],
                "geometry": {"length": 2, "width": 0.2, "height": 0.4},
                },
            ),
        }

# surrounded
surrounded = {
        "simpleBox1": BoxObstacle(name="simpleBox1", content_dict={
            "movable": True,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [253, 1, 0],
            "position": [-2, 0, 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "simpleBox2": BoxObstacle(name="simpleBox2", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [160, 214, 54],
            "position": [2, 0, 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "simpleBox3": BoxObstacle(name="simpleBox3", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [238, 222, 4],
            "position": [1, math.sqrt(3), 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "simpleBox4": BoxObstacle(name="simpleBox4", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [247, 105, 21],
            "position": [-1, math.sqrt(3), 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "simpleBox5": BoxObstacle(name="simpleBox5", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [47, 162, 54],
            "position": [1, -math.sqrt(3), 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "simpleBox6": BoxObstacle(name="simpleBox6", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [51, 62, 212],
            "position": [-1, -math.sqrt(3), 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
}

# swap
box_dict = {
    "color": [140, 125, 125],
    "position": [2, 1, 1],
    "type": "box",
    "geometry": {"length": 0.6, "width": 0.5, "height": 0.6},
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
        "small_duck": UrdfObstacle(name="small_duck", content_dict=duck_small_dict)}
