import pytest

import numpy as np

from robot_brain.global_planning.hgraph.local_planning.sample_based.push_motion_planner import PushMotionPlanner
from motion_planning_env.box_obstacle import BoxObstacle
from robot_brain.state import State
from robot_brain.obstacle import Obstacle

def test_init():

    box_dict = {
        "movable": True,
        "orientation": [0, 0, 3.1415/2],
        "mass": 3,
        "type": "box",
        "color": [0/255, 255/255, 0/255, 1],
        "position": [-2, -1.2, 0.5],
        "geometry": {"length": 1.4, "width": 1.4, "height": 0.3},
    }

    box_obst = Obstacle("box", State(), BoxObstacle(name="box", content_dict=box_dict))

    pmp = PushMotionPlanner(
        grid_x_length= 10,
        grid_y_length= 10,
        obstacles= {},
        obstacle= box_obst,
        step_size= 0.4,
        search_size= 0.5,
        configuration_grid_map=None)

    pmp.shortest_path(State(), State(pos=np.array([4,5,10])))

    assert False

