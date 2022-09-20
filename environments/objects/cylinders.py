from motion_planning_env.cylinder_obstacle import CylinderObstacle

cylinder_dict = {
    "movable": True,
    "mass": 1,
    "type": "cylinder",
    "color": [0/255, 255/255, 0/255, 1],
    "position": [-1.0, 3.0, 1.0],
    "geometry": {"radius": 0.6, "height": 0.1},
}

cylinder = CylinderObstacle(name="simple_cilinder", content_dict=cylinder_dict)
