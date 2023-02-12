import math
from motion_planning_env.box_obstacle import BoxObstacle

blocking_object = BoxObstacle(name="blocking_object", content_dict={
            "movable": True,
            "mass": 400,
            "type": "box",
            "orientation": [0,0,0],
            "color": [145, 151, 127, 1],
            "position":[0, 1.1, 0.5],
            "geometry": {"length": 1, "width": 1, "height": 0.6},
        })

wall1 =  BoxObstacle(name="wall1", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,math.pi/2],
            "color": [245, 181, 27, 1],
            "position": [-1, -1.5, 0.2],
            "geometry": {"length": 6, "width": 0.2, "height": 0.4},
        })

wall2 = BoxObstacle(name="wall2", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,math.pi/2],
            "color": [245, 181, 27, 1],
            "position": [1, -1.5, 0.2],
            "geometry": {"length": 6, "width": 0.2, "height": 0.4},
        })
