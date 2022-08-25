from MotionPlanningEnv.cylinderObstacle import CylinderObstacle

cylinder_dict = {
    "movable": True,
    "mass": 1000,
    "type": "cylinder",
    "color": [203/255, 210/255, 143/255, 1],
    "position": [-1.0, 3.0, 1.0],
    "geometry": {"radius": 0.6, "height": 1},
}

cylinder = CylinderObstacle(name="simpleCilinder", content_dict=cylinder_dict)
