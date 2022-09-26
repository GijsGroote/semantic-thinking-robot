from motion_planning_env.box_obstacle import BoxObstacle

box_dict = {
    "movable": True,
    "orientation": [1,1,1,1],
    "mass": 3,
    "type": "box",
    "color": [0/255, 255/255, 0/255, 1],
    "position": [2.0, 2.0, 1.0],
    "geometry": {"length": 0.5, "width": 0.4, "height": 0.3},
}

box = BoxObstacle(name="simple_box", content_dict=box_dict)
