import os
from motion_planning_env.urdf_obstacle import UrdfObstacle

# cute duck
urdf_duck_dict= {
    "type": "urdf",
    "position": [2.0, 0.0, 0.25],
    "geometry": {
        "urdf": os.path.join(os.path.dirname(__file__), "obstacle_data/duck/duck.urdf"),
    }
}

urdf_duck = UrdfObstacle(name="duckUrdf", content_dict=urdf_duck_dict)

# cute duck1
urdf_duck1_dict= {
    "type": "urdf",
    "position": [2.0, -1.0, 0.25],
    "geometry": {
         "urdf": os.path.join(os.path.dirname(__file__), "obstacle_data/duck/duck.urdf"),
    }
}

urdf_duck1 = UrdfObstacle(name="duckUrdf", content_dict=urdf_duck1_dict)

# huge duck
urdf_duck2_dict= {
    "type": "urdf",
    "position": [4.0, 15.0, 7.2],
    "geometry": {
        "urdf": os.path.join(os.path.dirname(__file__), "obstacle_data/duck/duck.urdf"),
    }
}

urdf_duck2 = UrdfObstacle(name="duck_urdf", content_dict=urdf_duck2_dict)
