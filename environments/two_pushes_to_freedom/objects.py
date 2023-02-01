import math
from motion_planning_env.box_obstacle import BoxObstacle

blocking_object1 = BoxObstacle(name="blocking_object1", content_dict={
            "movable": True,
            "mass": 400,
            "type": "box",
            "orientation": [0,0,0],
            "color": [145, 151, 127, 1],
            "position":[0, -2.1, 0.5],
            "geometry": {"length": 1.5, "width": 2.2, "height": 0.6},
        })
blocking_object2 = BoxObstacle(name="blocking_object2", content_dict={
            "movable": True,
            "mass": 400,
            "type": "box",
            "orientation": [0,0,0],
            "color": [145, 151, 127, 1],
            "position":[-3, 0, 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 0.6},
        })

center_wall =  BoxObstacle(name="center_wall", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "color": [245, 181, 27, 1],
            "position": [-1.5, -1.6, 0.2],
            "geometry": {"length": 1, "width": 3, "height": 0.4},
        })
wall1 =  BoxObstacle(name="wall1", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "color": [245, 181, 27, 1],
            "position": [-4.5, 0, 0.2],
            "geometry": {"length": 1, "width": 0.2, "height": 0.4},
        })

wall2 = BoxObstacle(name="wall2", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,0],
            "color": [245, 181, 27, 1],
            "position":[-4.5, 1.5, 0.2],
            "geometry": {"length": 1, "width": 0.2, "height": 0.4},
        })

wall3 = BoxObstacle(name="wall3", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,math.pi/2],
            "color": [245, 181, 27, 1],
            "position": [-4, 2.25, 0.2],
            "geometry": {"length": 1.7, "width": 0.2, "height": 0.4},
        })

wall4 = BoxObstacle(name="wall4", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,0],
            "color": [245, 181, 27, 1],
            "position": [-3, 3, 0.2],
            "geometry": {"length": 2.2, "width": 0.2, "height": 0.4},
        })

wall5 = BoxObstacle(name="wall5", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,math.pi/2],
            "color": [245, 181, 27, 1],
            "position": [-2, 2, 0.2],
            "geometry": {"length": 2.2, "width": 0.2, "height": 0.4},
        })

wall6 =  BoxObstacle(name="wall6", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "color": [245, 181, 27, 1],
            "position": [-.5, 1, 0.2],
            "geometry": {"length": 3.2, "width": 0.2, "height": 0.4},
        })

wall7 = BoxObstacle(name="wall7", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,math.pi/2],
            "color": [245, 181, 27, 1],
            "position": [1, -3, 0.2],
            "geometry": {"length": 8.2, "width": 0.2, "height": 0.4},
        })

wall8 = BoxObstacle(name="wall8", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,0],
            "color": [245, 181, 27, 1],
            "position":[0, -7, 0.2],
            "geometry": {"length": 2.2, "width": 0.2, "height": 0.4},
        })

wall9 = BoxObstacle(name="wall9", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,math.pi/2],
            "color": [245, 181, 27, 1],
            "position": [-1, -5.5, 0.2],
            "geometry": {"length": 3.2, "width": 0.2, "height": 0.4},
        })

wall10 = BoxObstacle(name="wall10", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,0],
            "color": [245, 181, 27, 1],
            "position": [-2.5, -4, 0.2],
            "geometry": {"length": 3.2, "width": 0.2, "height": 0.4},
        })


wall11 = BoxObstacle(name="wall11", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0,0,math.pi/2],
            "color": [245, 181, 27, 1],
            "position": [-4, -2, 0.2],
            "geometry": {"length": 4.2, "width": 0.2, "height": 0.4},
        })
