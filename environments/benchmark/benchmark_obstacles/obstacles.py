import os
import math
from motion_planning_env.urdf_obstacle import UrdfObstacle
from motion_planning_env.box_obstacle import BoxObstacle

# blockade
pushable_cube_dict = {
    "movable": True,
    "orientation": [0, 0, 0.2],
    "mass": 3,
    "color": [22, 63, 88],
    "position": [1, 2, 0.8],
    "geometry": {"length": 0.8, "width": 0.8, "height": 0.8},
}
pushable_cube = BoxObstacle(name="simpeBox", content_dict=pushable_cube_dict)

urdf_duck_dict= {
    "position": [2, 0, 0.25],
    "orientation": [math.pi/2, 0, 0],
    "geometry": {
        "urdf": os.path.join(os.path.dirname(__file__), "./obstacle_data/duck/duck.urdf"),
    }
}
urdf_duck = UrdfObstacle(name="duck_urdf", content_dict=urdf_duck_dict)

dead_end = {
        "wall1": BoxObstacle(name="simpeBox", content_dict={
                "movable": False,
                "orientation": [0, 0, math.pi/2],
                "color": [245, 181, 27],
                "position": [3.9, 0, 0.2],
                "geometry": {"length": 1.8, "width": 0.2, "height": 0.4},
            }),
        "wall2": BoxObstacle(name="simpeBox", content_dict={
                "movable": False,
                "orientation": [0,0,0],
                "color": [245, 181, 27, 1],
                "position":[3, 1, 0.2],
                "geometry": {"length": 2, "width": 0.2, "height": 0.4},
            }),
        "wall3": BoxObstacle(name="simpeBox", content_dict={
                "movable": False,
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
        "simpleBox2": BoxObstacle(name="simlpeBox2", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [160, 214, 54],
            "position": [2, 0, 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "simpleBox3": BoxObstacle(name="simlpeBox3", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [238, 222, 4],
            "position": [1, math.sqrt(3), 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "simpleBox4": BoxObstacle(name="simlpeBox4", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [247, 105, 21],
            "position": [-1, math.sqrt(3), 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "simpleBox5": BoxObstacle(name="simlpeBox5", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [47, 162, 54],
            "position": [1, -math.sqrt(3), 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "simpleBox6": BoxObstacle(name="simlpeBox6", content_dict={
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
    "geometry": {"length": 0.6, "width": 0.5, "height": 0.6},
}
box_small = BoxObstacle(name="simple_box", content_dict=box_dict)

duck_small_dict= {
    "position": [2, -1, 0.25],
    "orientation": [math.pi/2, 0, 0],
    "geometry": {
        "urdf": os.path.join(os.path.dirname(__file__), "./obstacle_data/duck/duck.urdf"),
    }
}
duck_small = UrdfObstacle(name="duck_urdf", content_dict=duck_small_dict)
