import math
from motion_planning_env.box_obstacle import BoxObstacle

# surrounded
surrounded = {
        "simpleBox1": BoxObstacle(name="simpleBox1", content_dict={
            "movable": False,
            # "movable": True,
            # "mass": 3,
            "type": "box",
            "orientation": [0, 0, 0],
            "color": [253, 1, 0],
            "position": [-2, 0, 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1.0},
            }),
        "simpleBox2": BoxObstacle(name="simpleBox2", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [160, 214, 54],
            "position": [2, 0, 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1},
            }),
        "simpleBox3": BoxObstacle(name="simpleBox3", content_dict={
            # "movable": False,
            "movable": True,
            "mass": 3,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [238, 222, 4],
            "position": [0.9, math.sqrt(3), 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1},
            }),
        "simpleBox4": BoxObstacle(name="simpleBox4", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [255, 165, 0],
            "position": [-0.9, math.sqrt(3), 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1},
            }),
        "simpleBox5": BoxObstacle(name="simpleBox5", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [47, 1.5, 54],
            "position": [0.8, -math.sqrt(3), 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1},
            }),
        "simpleBox6": BoxObstacle(name="simpleBox6", content_dict={
            "movable": False,
            "type": "box",
            "orientation": [0, 0, 0],
            "mass": 3,
            "color": [51, 62, 212],
            "position": [-0.9, -math.sqrt(3), 0.5],
            "geometry": {"length": 1.5, "width": 1.5, "height": 1},
            }),
}
