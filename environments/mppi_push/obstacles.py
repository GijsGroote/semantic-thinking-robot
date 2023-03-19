from motion_planning_env.box_obstacle import BoxObstacle

box_dict = {
    "movable": True,
    "orientation": [0, 0, 3.1415/2],
    "mass": 3,
    "type": "box",
    "color": [0/255, 255/255, 0/255, 1],
    "position": [0, 2, 0.5],
    "geometry": {"length": 2, "width": 2, "height": 0.3},
}

box = BoxObstacle(name="simple_box", content_dict=box_dict)


