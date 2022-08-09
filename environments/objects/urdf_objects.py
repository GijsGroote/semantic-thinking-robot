from MotionPlanningEnv.urdfObstacle import UrdfObstacle

import os

# cute duck
urdf_duck_dict= {
    'dim': 3,
    'type': 'urdf',
    'geometry': {'position': [2.0, 0.0, 0.25]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/duck/duck.urdf'),
}

urdf_duck = UrdfObstacle(name='duckUrdf', contentDict=urdf_duck_dict)

# cute duck2
urdf_duck2_dict= {
    'dim': 3,
    'type': 'urdf',
    'geometry': {'position': [2.0, -1.0, 0.25]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/duck/duck.urdf'),
}

# urdf_duck2 = UrdfObstacle(name='duckUrdf', contentDict=urdf_duck2_dict)

# huge duck 
urdf_duck2_dict= {
    'dim': 3,
    'type': 'urdf',
    'geometry': {'position': [4.0, 15.0, 7.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/duck/duck.urdf'),
}

urdf_duck2 = UrdfObstacle(name='duckUrdf', contentDict=urdf_duck2_dict)
