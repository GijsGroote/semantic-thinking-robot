import numpy as np
from MotionPlanningEnv.boxObstacle import BoxObstacle
from MotionPlanningEnv.cylinderObstacle import CylinderObstacle
from MotionPlanningEnv.sphereObstacle import SphereObstacle

# sphere_dict = {
#     "movable": True,
#     "mass": 1,
#     "type": "sphere",
#     "color": [227/255, 180/255, 72/255, 1],
#     "position": [1.0, 1.0, 1.0],
#     "geometry": {"radius": 0.6},
# }
# sphere = SphereObstacle(name="simpeSphere", content_dict=sphere_dict)
#
# box_dict = {
#     "movable": True,
#     "orientation": [1,1,1,1],
#     "mass": 3,
#     "type": "box",
#     "color": [58/255, 107/255, 52/255, 1],
#     "position": [2.0, 2.0, 1.0],
#     "geometry": {"length": 0.5, "width": 0.4, "height": 0.3},
# }
# box = BoxObstacle(name="simpeBox", content_dict=box_dict)
#
# cylinder_dict = {
#     "movable": True,
#     "mass": 1000,
#     "type": "cylinder",
#     "color": [203/255, 210/255, 143/255, 1],
#     "position": [-1.0, 3.0, 1.0],
#     "geometry": {"radius": 0.6, "height": 1},
# }
# cylinder = CylinderObstacle(name="simpleCilinder", content_dict=cylinder_dict)

# blockade
pushable_cube_dict = {
    "movable": True,
    "orientation": [1,1,1,1],
    "mass": 3,
    "type": "box",
    "color": [58/255, 107/255, 52/255, 1],
    "position": [1.0, 2.0, 0.3],
    "geometry": {"length": 0.3, "width": 0.3, "height": 0.3},
}
pushable_cube = BoxObstacle(name="simpeBox", content_dict=pushable_cube_dict)

dead_end1_dict = {
    "movable": False,
    "orientation": [1,1,1,1],
    "type": "box",
    "color": [58/255, 107/255, 52/255, 1],
    "position": [4.0, 0.0, 0.0],
    "geometry": {"length": 0.2, "width": 2.2, "height": 0.5},
}
dead_end2_dict = {
    "movable": False,
    "orientation": [1,1,1,1],
    "type": "box",
    "color": [58/255, 107/255, 52/255, 1],
    "position":[2.8, 1.0, np.pi/2],
    "geometry": {"length": 0.2, "width": 2.2, "height": 0.5},
}
dead_end3_dict = {
    "movable": False,
    "orientation": [1,1,1,1],
    "type": "box",
    "color": [58/255, 107/255, 52/255, 1],
    "position": [2.8, -1.0, np.pi/2],
    "geometry": {"length": 0.2, "width": 2.2, "height": 0.5},
}
dead_end = {
        "wall1", BoxObstacle(name="simpeBox", content_dict=dead_end1_dict),
        "wall2", BoxObstacle(name="simpeBox", content_dict=dead_end2_dict),
        "wall3", BoxObstacle(name="simpeBox", content_dict=dead_end3_dict),
        }
# surround

# swap
