from motion_planning_env.cylinder_obstacle import CylinderObstacle

cylinder_dict = {
    "movable": True,
    "mass": 1,
    "type": "cylinder",
    "color": [0/255, 255/255, 0/255, 1],
    "position": [-2, -1, 0.5],
    "geometry": {"radius": 0.92, "height": 0.25},
}

cylinder = CylinderObstacle(name="simple_cilinder", content_dict=cylinder_dict)
