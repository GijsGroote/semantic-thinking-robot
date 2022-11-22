from motion_planning_env.box_obstacle import BoxObstacle

box_dict = {
    "movable": True,
    "orientation": [0, 0, 3.1415/2],
    "mass": 3,
    "type": "box",
    "color": [0/255, 255/255, 0/255, 3],
    "position": [-2.6, 2.2, 0.5],
    "geometry": {"length": 1, "width": 4.0, "height": 0.3},
}

box = BoxObstacle(name="simple_box", content_dict=box_dict)

box_dict = {
    "movable": True,
    "orientation": [0, 0, 0.4],
    "mass": 3,
    "type": "box",
    "color": [0/255, 255/255, 0/255, 3],
    "position": [3.0, -2.0, 0.5],
    "geometry": {"length": 6, "width": 2.0, "height": 0.3},
}

box2 = BoxObstacle(name="simple_box2", content_dict=box_dict)
