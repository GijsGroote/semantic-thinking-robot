import numpy as np

pushable_cube = {
        "dim": np.array([1.4, 1.4, 1.4]),
        "poses_2d": [
            [1, 2, 0.2],
            ]
        }

dead_end = {
        "wall1": {
            "dim": np.array([0.2, 2.2, 0.5]),
            "poses_2d": [
                [4, 0, 0],
                ]
            },
        "wall2": {
            "dim": np.array([0.2, 2.2, 0.5]),
            "poses_2d": [
                [2.8, 1, np.pi/2],
                ]
            },
        "wall3": {
            "dim": np.array([0.2, 2.2, 0.5]),
            "poses_2d": [
                [2.8, -1, np.pi/2],
                ]
            },
        }


