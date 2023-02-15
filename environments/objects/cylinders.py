from motion_planning_env.cylinder_obstacle import CylinderObstacle

cylinder_dict = {
    "movable": True,
    "mass": 1000,
    "type": "cylinder",
    "color": [0/255, 255/255, 0/255, 1],
    "position": [-1, -1, 0.5],
    "geometry": {"radius": 0.5, "height": 0.25},
}

cylinder = CylinderObstacle(name="simple_cilinder", content_dict=cylinder_dict)
