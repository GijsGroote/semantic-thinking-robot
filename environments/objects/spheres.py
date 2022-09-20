from motion_planning_env.sphere_obstacle import SphereObstacle

sphere_dict = {
    "movable": True,
    "mass": 1,
    "type": "sphere",
    "color": [0/255, 255/255, 0/255, 1],
    "position": [1.0, 1.0, 1.0],
    "geometry": {"radius": 0.6},
}

sphere = SphereObstacle(name="simpe_sphere", content_dict=sphere_dict)

