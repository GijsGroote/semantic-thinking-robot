from MotionPlanningEnv.sphereObstacle import SphereObstacle

sphere_dict = {
    "movable": True,
    "mass": 1, 
    "type": "sphere",
    "color": [227/255, 180/255, 72/255, 1],
    "position": [1.0, 1.0, 1.0], 
    "geometry": {"radius": 0.6},
}

sphere = SphereObstacle(name="simpeSphere", content_dict=sphere_dict)

