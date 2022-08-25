from MotionPlanningEnv.boxObstacle import BoxObstacle 

box_dict = {
    "movable": True,
    "orientation": [1,1,1,1],
    "mass": 3, 
    "type": "box",
    "color": [58/255, 107/255, 52/255, 1],
    "position": [2.0, 2.0, 1.0], 
    "geometry": {"length": 0.5, "width": 0.4, "heigth": 0.3},
}

box = BoxObstacle(name="simpeBox", content_dict=box_dict)


