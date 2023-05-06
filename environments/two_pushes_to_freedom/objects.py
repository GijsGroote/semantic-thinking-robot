import math
from motion_planning_env.box_obstacle import BoxObstacle
from helper_functions.geometrics import to_interval_zero_to_two_pi, rotate_obst_around_origin



def create_two_pushes_to_freedom_objects(rotate_by_theta=0):
    """ create objects for two pushes to freedom environment. """

    rotate_by_theta = to_interval_zero_to_two_pi(rotate_by_theta)

    (pos, orien) = rotate_obst_around_origin([0, -2.1, 0.5], [0, 0, 0], rotate_by_theta)
    blocking_object1 = BoxObstacle(name="blocking_object1", content_dict={
                "movable": True,
                "mass": 400,
                "type": "box",
                "orientation": orien,
                "color": [145, 151, 127, 1],
                "position": pos,
                "geometry": {"length": 1.5, "width": 2.2, "height": 0.6},
            })

    (pos, orien) = rotate_obst_around_origin([-3, 0.5, 0.5], [0, 0, 0], rotate_by_theta)
    blocking_object2 = BoxObstacle(name="blocking_object2", content_dict={
                "movable": True,
                "mass": 400,
                "type": "box",
                "orientation": orien,
                "color": [145, 151, 127, 1],
                "position": pos,
                "geometry": {"length": 1.5, "width": 1.5, "height": 0.6},
            })

    (pos, orien) = rotate_obst_around_origin([-1.5, -1.6, 0.2], [0, 0, 0], rotate_by_theta)
    center_wall =  BoxObstacle(name="center_wall", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 1, "width": 3, "height": 0.4},
            })
    (pos, orien) = rotate_obst_around_origin([-4.5, 0, 0.2], [0, 0, 0], rotate_by_theta)
    wall1 =  BoxObstacle(name="wall1", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 1.2, "width": 0.2, "height": 0.4},
            })
    (pos, orien) = rotate_obst_around_origin([-4.5, 1.5, 0.2], [0, 0, 0], rotate_by_theta)
    wall2 = BoxObstacle(name="wall2", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 1.2, "width": 0.2, "height": 0.4},
            })
    (pos, orien) = rotate_obst_around_origin([-4, 2.25, 0.2], [0, 0, math.pi/2], rotate_by_theta)
    wall3 = BoxObstacle(name="wall3", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 1.7, "width": 0.2, "height": 0.4},
            })
    (pos, orien) = rotate_obst_around_origin([-3, 3, 0.2], [0, 0, 0], rotate_by_theta)
    wall4 = BoxObstacle(name="wall4", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 2.2, "width": 0.2, "height": 0.4},
            })
    (pos, orien) = rotate_obst_around_origin([-2, 2, 0.2], [0, 0, math.pi/2], rotate_by_theta)
    wall5 = BoxObstacle(name="wall5", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 2.2, "width": 0.2, "height": 0.4},
            })
    (pos, orien) = rotate_obst_around_origin([-.5, 1, 0.2], [0, 0, 0], rotate_by_theta)
    wall6 =  BoxObstacle(name="wall6", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 3.2, "width": 0.2, "height": 0.4},
            })

    (pos, orien) = rotate_obst_around_origin([1, -3, 0.2], [0, 0, math.pi/2], rotate_by_theta)
    wall7 = BoxObstacle(name="wall7", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 8.2, "width": 0.2, "height": 0.4},
            })

    (pos, orien) = rotate_obst_around_origin([0, -7, 0.2], [0, 0, 0], rotate_by_theta)
    wall8 = BoxObstacle(name="wall8", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 2.2, "width": 0.2, "height": 0.4},
            })

    (pos, orien) = rotate_obst_around_origin([-1, -5.5, 0.2], [0, 0, math.pi/2], rotate_by_theta)
    wall9 = BoxObstacle(name="wall9", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 3.2, "width": 0.2, "height": 0.4},
            })

    (pos, orien) = rotate_obst_around_origin([-2.5, -4, 0.2], [0, 0, 0], rotate_by_theta)
    wall10 = BoxObstacle(name="wall10", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 3.2, "width": 0.2, "height": 0.4},
            })
    (pos, orien) = rotate_obst_around_origin([-4, -2, 0.2], [0, 0, math.pi/2], rotate_by_theta)
    wall11 = BoxObstacle(name="wall11", content_dict={
                "movable": False,
                "type": "box",
                "orientation": orien,
                "color": [245, 181, 27, 1],
                "position": pos,
                "geometry": {"length": 4.2, "width": 0.2, "height": 0.4},
            })

    return {blocking_object1.name(): blocking_object1,
            blocking_object2.name(): blocking_object2,
            center_wall.name(): center_wall,
            wall1.name(): wall1,
            wall2.name(): wall2,
            wall3.name(): wall3,
            wall4.name(): wall4,
            wall5.name(): wall5,
            wall6.name(): wall6,
            wall7.name(): wall7,
            wall8.name(): wall8,
            wall9.name(): wall9,
            wall10.name(): wall10,
            wall11.name(): wall11}
