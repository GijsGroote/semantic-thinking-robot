from robot_brain.global_planning.hgraph.boxer_robot_hgraph import BoxerRobotHGraph
import numpy as np


from robot_brain.object import Object
from robot_brain.state import State
from robot_brain.global_planning.conf_set_node import ConfSetNode
from robot_brain.global_planning.object_set_node import ObjectSetNode
from robot_brain.global_planning.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.global_planning.edge import Edge

def main():
    
    robot = Object(
        "boxer_robot",
        State(
            pos=np.array([1,2,3]),
            vel=np.array([0,0,0]),
        ),
        "urdf",
    )

    hgraph = BoxerRobotHGraph(robot)

    hgraph.add_start_node(ObjectSetNode(1, "start node", []))
    hgraph.add_node(ObjectSetNode(4, "generated node", []))
    hgraph.add_target_node(ConfSetNode(3, "target node", []))


    node2 = ObjectSetNode(2, "current node", [])
    hgraph.current_node = node2
    hgraph.add_start_node(node2)
    hgraph.add_edge(Edge("id", 4, 3, "transition, no planning completed", "controller"))
    hgraph.add_edge(Edge("id", 2, 4, "transition, no planning completed", "controller"))
    hgraph.add_edge(Edge("id", 1, 2, "transition, motion planning completed", "controller", True))

    hgraph.visualise()

if __name__ == "__main__":
    # 8 stages of the hgraph, in
    main()

