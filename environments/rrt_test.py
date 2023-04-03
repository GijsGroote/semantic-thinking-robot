# import bisect
import numpy as np
from sortedcontainers import SortedDict
from abc import abstractmethod
from robot_brain.object import Object
from motion_planning_env.cylinder_obstacle import CylinderObstacle 
from robot_brain.state import State
from robot_brain.global_planning.hgraph.local_planning.sample_based.drive_motion_planner import DriveMotionPlanner
from robot_brain.global_planning.hgraph.local_planning.sample_based.motion_planner import ConnectivityGraph

def main():



    # conn_graph = ConnectivityGraph([1.1, 4.2], [2.1, 5.4])
    #
    # conn_graph.add_sample((0.1, 0.2), 0, 1)
    # conn_graph.add_sample((0.01, 0.4423423), 0, 1)
    # conn_graph.add_sample((0.2, 0.42423), 0, 2)
    # conn_graph.add_sample((0.4, 0.4453654), 0, 1)
    # conn_graph.add_sample((0.5, 0.74576), 1, 3)
    # conn_graph.add_sample((0.6, 0.424), 1, 1)
    #
    # print(conn_graph.samples) 
    # print(conn_graph.get_closest_sample((0.11, 1.0)))




    cylinder_dict = {

        "type": "cylinder",
        "position": [0, 0, 0],
        "geometry": {"radius": 0.5, "height": 0.2},
        }

    robot_properties = CylinderObstacle(name="cylinder_robot", content_dict=cylinder_dict)


    obst = Obstacle(
        name="test_test_test",
        state=State(),
        properties=robot_properties,
        )

    dmp = DriveMotionPlanner(
            grid_x_length = 10.0,
            grid_y_length = 10.0,
            objects = {},
            object = obst,
            step_size = 0.5,
            search_size = 1)

    dmp.search(State(pos=np.array([-2, 3, 0])), State(pos=np.array([2,3,0])))

    dmp.connectivity_graph.visualise()


















if __name__ == "__main__":
    main()
