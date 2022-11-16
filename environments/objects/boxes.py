from motion_planning_env.box_obstacle import BoxObstacle

box_dict = {
    "movable": True,
    "orientation": [0, 0, 0.1],
    "mass": 3,
    "type": "box",
    "color": [0/255, 255/255, 0/255, 3],
    "position": [-1.0, 2.0, 0.5],
    "geometry": {"length": 1, "width": 0.4, "height": 0.3},
}

box = BoxObstacle(name="simple_box", content_dict=box_dict)
