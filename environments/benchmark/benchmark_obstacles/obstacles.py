import os
import math
from motion_planning_env.urdf_obstacle import UrdfObstacle
from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle
from motion_planning_env.sphere_obstacle import SphereObstacle

surrounded = {
        "box1": BoxObstacle(name="simpeBox", content_dict={
            "movable": True,
            "orientation": [0, 0, 0],
            "mass": 3,
            "type": "box",
            "color": [22, 63, 88, 1],
            "position": [-2, 0, 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "box2": BoxObstacle(name="simpeBox", content_dict={
            "movable": True,
            "orientation": [0, 0, 0],
            "mass": 3,
            "type": "box",
            "color": [22, 63, 88, 1],
            "position": [2, 0, 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "box3": BoxObstacle(name="simpeBox", content_dict={
            "movable": True,
            "orientation": [0, 0, 0],
            "mass": 3,
            "type": "box",
            "color": [22, 63, 88, 1],
            "position": [1, math.sqrt(3), 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "box4": BoxObstacle(name="simpeBox", content_dict={
            "movable": True,
            "orientation": [0, 0, 0],
            "mass": 3,
            "type": "box",
            "color": [22, 63, 88, 1],
            "position": [-1, math.sqrt(3), 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "box5": BoxObstacle(name="simpeBox", content_dict={
            "movable": True,
            "orientation": [0, 0, 0],
            "mass": 3,
            "type": "box",
            "color": [22, 63, 88, 1],
            "position": [1, -math.sqrt(3), 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
        "box6": BoxObstacle(name="simpeBox", content_dict={
            "movable": True,
            "orientation": [0, 0, 0],
            "mass": 3,
            "type": "box",
            "color": [22, 63, 88, 1],
            "position": [-1, -math.sqrt(3), 0.6],
            "geometry": {"length": 1.2, "width": 1.2, "height": 1.2},
            }),
}

# blockade
pushable_cube_dict = {
    "movable": True,
    "orientation": [0, 0, 0.2],
    "mass": 3,
    "type": "box",
    "color": [22, 63, 88, 1],
    "position": [1.0, 2.0, 0.8],
    "geometry": {"length": 0.8, "width": 0.8, "height": 0.8},
}
pushable_cube = BoxObstacle(name="simpeBox", content_dict=pushable_cube_dict)

# duck
urdf_duck_dict= {
    "type": "urdf",
    "position": [2.0, 0.0, 0.25],
    "geometry": {
        "urdf": os.path.join(os.path.dirname(__file__), "./obstacle_data/duck/duck.urdf"),
    }
}

urdf_duck = UrdfObstacle(name="duck_urdf", content_dict=urdf_duck_dict)

dead_end1_dict = {
    "movable": False,
    "orientation": [0,0,math.pi/2],
    "type": "box",
    "color": [245.0, 181.0, 27.0, 1],
    "position": [3.9, 0.0, 0.2],
    "geometry": {"length": 1.8, "width": 0.2, "height": 0.4},
}

dead_end2_dict = {
    "movable": False,
    "orientation": [0,0,0],
    "type": "box",
    "color": [245, 181, 27, 1],
    "position":[3.0, 1.0, 0.2],
    "geometry": {"length": 2.0, "width": 0.2, "height": 0.4},
}
dead_end3_dict = {
    "movable": False,
    "orientation": [0,0,0],
    "type": "box",
    "color": [245/255, 181/255, 27/255, 1],
    "position": [3.0, -1.0, 0.2],
    "geometry": {"length": 2.0, "width": 0.2, "height": 0.4},
}
dead_end = {
        "wall1": BoxObstacle(name="simpeBox", content_dict=dead_end1_dict),
        "wall2": BoxObstacle(name="simpeBox", content_dict=dead_end2_dict),
        "wall3": BoxObstacle(name="simpeBox", content_dict=dead_end3_dict),
        }


# swap

# temp example files
sphere_dict = {
    "movable": True,
    "mass": 1,
    "type": "sphere",
    "color": [227/255, 180/255, 72/255, 1],
    "position": [1.0, 1.0, 1.0],
    "geometry": {"radius": 0.6},
}
sphere = SphereObstacle(name="simpeSphere", content_dict=sphere_dict)

box_dict = {
    "movable": True,
    "orientation": [1,1,1,1],
    "mass": 3,
    "type": "box",
    "color": [58/255, 107/255, 52/255, 1],
    "position": [2.0, 2.0, 1.0],
    "geometry": {"length": 0.5, "width": 0.4, "height": 0.3},
}
box = BoxObstacle(name="simpeBox", content_dict=box_dict)

cylinder_dict = {
    "movable": True,
    "mass": 1000,
    "type": "cylinder",
    "color": [203/255, 210/255, 143/255, 1],
    "position": [-1.0, 3.0, 3.0],
    "orientation": [math.pi/2,0,0],
    "geometry": {"radius": 1, "height": 1},
}
cylinder = CylinderObstacle(name="simpleCilinder", content_dict=cylinder_dict)


