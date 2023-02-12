import math
from motion_planning_env.box_obstacle import BoxObstacle

blocking_object = BoxObstacle(name="blocking_object", content_dict={
            "movable": True,
            "mass": 400,
            "type": "box",
            "orientation": [0,0,0],
            "color": [145, 151, 127, 1],
            "position":[-2, 0, 0.2],
            "geometry": {"length": 1.3, "width": 1.3, "height": 0.4},
        })

wall1 =  BoxObstacle(name="wall1", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, math.pi/2],
            "color": [245, 181, 27, 1],
            "position": [1.4, 0, 0.2],
            "geometry": {"length": 1.8, "width": 0.2, "height": 0.4},
        })

wall2 = BoxObstacle(name="wall2", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,0],
            "color": [245, 181, 27, 1],
            "position":[-0.5, 1, 0.2],
            "geometry": {"length": 4, "width": 0.2, "height": 0.4},
        })
wall3 = BoxObstacle(name="wall3", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,0],
            "color": [245, 181, 27, 1],
            "position": [-0.5, -1, 0.2],
            "geometry": {"length": 4, "width": 0.2, "height": 0.4},
        })
