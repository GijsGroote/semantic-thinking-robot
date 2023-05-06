from typing import Tuple

from motion_planning_env.box_obstacle import BoxObstacle
from motion_planning_env.sphere_obstacle import SphereObstacle
from motion_planning_env.cylinder_obstacle import CylinderObstacle


from robot_brain.object import Object

def get_max_dimension_from_object(obj: Object) -> float:
    """ return the maximal dimension for a box or cylinder object. """

    # retrieve min and max dimension of the object
    if isinstance(obj.properties, BoxObstacle):
        max_obj_dimension = max(obj.properties.width(), obj.properties.length())
    elif isinstance(obj.properties, CylinderObstacle):
        max_obj_dimension = obj.properties.radius()
    else:
        raise ValueError(f"object not recognised, type: {type(obj)}")

    return max_obj_dimension

def get_min_max_dimension_from_object(obj: Object) -> Tuple[float, float]:
    """ return the minimal and maximal dimensions for a box or cylinder object. """

    # retrieve min and max dimension of the object
    if isinstance(obj.properties, BoxObstacle):
        min_obj_dimension = min(obj.properties.width(), obj.properties.length())
        max_obj_dimension = max(obj.properties.width(), obj.properties.length())
    elif isinstance(obj.properties, CylinderObstacle):
        min_obj_dimension = max_obj_dimension = obj.properties.radius()
    else:
        raise ValueError(f"object not recognised, type: {type(obj)}")

    return (min_obj_dimension, max_obj_dimension)
