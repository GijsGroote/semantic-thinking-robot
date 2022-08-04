import pybullet as p

def create_shape(shape_type, dim, mass, poses_2d=None, place_heigth=None):
 """
    creates a pybullet shape.

    Parameters
    ----------

    shape_type: str
        options are:
            "GEOM_SPHERE",
            "GEOM_BOX",
            "GEOM_CYLINDER",
            "GEOM_CAPSULE"
        .
    dim: np.ndarray or list
        dimensions for the shape, dependent on the shape_type:
            GEOM_SPHERE,    dim=[radius]
            GEOM_BOX,       dim=[width, length, height]
            GEOM_CYLINDER,  dim=[radius, length]
            GEOM_CAPSULE,   dim=[radius, length]
    mass: float
        objects mass (default = 0 : fixed shape)
    poses_2d: list
        poses where the shape should be placed. Each element
        must be of form [x_position, y_position, orientation]
    place_height: float
        z_position of the center of mass
    """
    # create collisionShape
    if shape_type == "GEOM_SPHERE":
        # check dimensions
        dim = filter_shape_dim(
                dim, "GEOM_SPHERE", 1, default=np.array([0.5])
                )
        shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=dim[0])
        default_height = dim[0]

    elif shape_type == "GEOM_BOX":
        if dim is not None:
            dim = 0.5 * dim
        # check dimensions
        dim = filter_shape_dim(
                dim, "GEOM_BOX", 3, default=np.array([0.5, 0.5, 0.5])
                )
        shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=dim)
        default_height = dim[2]

    elif shape_type == "GEOM_CYLINDER":
        # check dimensions
        dim = filter_shape_dim(
                dim, "GEOM_CYLINDER", 2, default=np.array([0.5, 1.0])
                )
        shape_id = p.createCollisionShape(
                p.GEOM_CYLINDER, radius=dim[0], height=dim[1]
                )
        default_height = 0.5 * dim[1]

    elif shape_type == "GEOM_CAPSULE":
        # check dimensions
        dim = filter_shape_dim(
                dim, "GEOM_CAPSULE", 2, default=np.array([0.5, 1.0])
                )
        shape_id = p.createCollisionShape(
                p.GEOM_CAPSULE, radius=dim[0], height=dim[1]
                )
        default_height = dim[0] + 0.5 * dim[1]

    else:
        warnings.warn(
                "Unknown shape type: {shape_type}, aborting..."
                )
        return

    if place_height is None:
        place_height = default_height

    # place the shape at poses_2d
    for pose in poses_2d:
        p.createMultiBody(
                baseMass=mass,
                baseCollisionShapeIndex=shape_id,
                baseVisualShapeIndex=shape_id,
                basePosition=[pose[0], pose[1], place_height],
                baseOrientation=p.getQuaternionFromEuler([0, 0, pose[2]]),
                )



def filter_shape_dim(dim: np.ndarray, shape_type: str, dim_len: int, default: np.ndarray) -> np.ndarray:
    """
    Checks and filters the dimension of a shape depending
    on the shape, warns were necessary.

    Parameters
    ----------

    dim: the dimension of the shape
    shape_type: the shape type
    dim_len: the number of dimensions should equal dim_len
    default: fallback option for dim

    """

    # check dimensions
    if isinstance(dim, np.ndarray) and np.size(dim) is dim_len:
        pass
    elif dim is None:
        dim = default
    else:
        warnings.warn(
            f"{shape_type} dimension should be of"
            "type (np.ndarray, list) with shape = ({dim_len}, )\n"
            " currently type(dim) = {type(dim)}. Aborting..."
        )
        return default
    return dim


