from MotionPlanningEnv.boxObstacle import BoxObstacle 

box_dict = {
    "dim": 3,
    "movable": True,
    "type": "box",
    "geometry": {"position": [1.0, 2.0, 1.0], "lwh": [0.7, 1, 1]},
}

box = BoxObstacle(name="simpeBox", contentDict=box_dict)


