from MotionPlanningEnv.sphereObstacle import SphereObstacle

sphere_dict = {
    "dim": 3,
    "movable": True,
    "type": "sphere",
    "geometry": {"position": [1.0, 2.0, 1.0], "radius": 0.7},
}

sphere = SphereObstacle(name="simpleSphere", contentDict=sphere_dict)


sphere_small_dict = {
        "dim": 3,
        "movable": True,
        "type": "sphere",
        "geometry": {"position": [-2.0, 1.0, 0.3], "radius": 0.2}

}

sphere_small = SphereObstacle(name="simpleSphere", contentDict=sphere_small_dict)


