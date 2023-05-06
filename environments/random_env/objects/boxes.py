from motion_planning_env.box_obstacle import BoxObstacle

box_dict = {
    "movable": True,
    "orientation": [0, 0, 3.1415/2],
    "mass": 3,
    "type": "box",
    "color": [0/255, 255/255, 0/255, 1],
    "position": [-2, -1, 0.5],
    "geometry": {"length": 1.4, "width": 1.4, "height": 0.3},
}

# TODO: give box a unique name
rand_box = BoxObstacle(name="rand_box", content_dict=box_dict)
