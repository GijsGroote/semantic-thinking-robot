from robot_brain.global_planning.hgraph.boxer_robot_hgraph import BoxerRobotHGraph
from robot_brain.global_planning.state_node import StateNode
from robot_brain.global_planning.hgraph.object_node import ObjectNode
from robot_brain.global_planning.hgraph.edge import Edge


def main(stage):

    hgraph = BoxerRobotHGraph()

    hgraph.addStartNode(ObjectNode(1, "start robot", []))
    hgraph.addTargetNode(StateNode(3, "target cube", []))
    hgraph.addStartNode(ObjectNode(2, "R: start cube", []))

    if stage==1:

        node1 = ObjectNode(1, "start robot", [])
        hgraph.current_node = node1
        hgraph.addStartNode(node1)
        hgraph.addNode(ObjectNode(4, "RM: cube", []))
        hgraph.addEdge(Edge("id", 1, 2, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 4, 3, "MPPI", "controller"))
        hgraph.addEdge(Edge("id", 2, 4, "Forward Model", "controller", True))

    if stage==2:

        node4 = ObjectNode(4, "RM: cube", [])
        hgraph.current_node = node4
        hgraph.addNode(node4)
        hgraph.addEdge(Edge("id", 1, 2, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 4, 3, "MPPI", "controller"))
        hgraph.addEdge(Edge("id", 2, 4, "Forward Model", "controller", True))


    if stage>=3 and stage<5:

        node4 = ObjectNode(4, "RM: cube", [])
        if stage == 3:
            hgraph.current_node = node4
        hgraph.addNode(node4)

        hgraph.addNode(StateNode(5, "RM: cube", []))
        hgraph.addNode(StateNode(6, "R: target green wall", []))
        node7 = StateNode(7, "RM: green wall", [])
        hgraph.addNode(node7)
        hgraph.addNode(StateNode(8, "R: green wall", []))
        hgraph.addEdge(Edge("id", 1, 2, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 5, 3, "MPPI", "controller"))
        hgraph.addEdge(Edge("id", 2, 4, "Forward Model", "controller", True))
        hgraph.addEdge(Edge("id", 4, 8, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 8, 7, "model fitting", "controller", True))
        hgraph.addEdge(Edge("id", 7, 6, "MPC", "controller"))
        hgraph.addEdge(Edge("id", 6, 5, "MPC", "controller"))

    if stage == 4:
        hgraph.current_node = node7

    if stage == 5:
        node4 = ObjectNode(4, "RM: cube", [])
        hgraph.addNode(node4)
        hgraph.addNode(StateNode(5, "RM: cube", []))
        hgraph.addNode(StateNode(6, "(aborted) R: target green wall ", []))
        node7 = ObjectNode(7, "RM: green wall", [])
        hgraph.current_node = node7
        hgraph.addNode(node7)
        hgraph.addNode(StateNode(8, "R: green wall", []))
        hgraph.addEdge(Edge("id", 1, 2, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 5, 3, "MPPI", "controller"))
        hgraph.addEdge(Edge("id", 2, 4, "Forward Model", "controller", True))
        hgraph.addEdge(Edge("id", 4, 8, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 8, 7, "model fitting", "controller", True))
        hgraph.addEdge(Edge("id", 6, 5, "MPC", "controller"))
        hgraph.addEdge(Edge("id", 7, 3, "MPPI", "controller"))

    if stage==6:
        node4 = ObjectNode(4, "RM: cube", [])
        hgraph.addNode(node4)
        hgraph.addNode(StateNode(5, "RM: cube", []))
        hgraph.addNode(StateNode(6, "(aborted) R: target green wall", []))
        node7 = ObjectNode(7, "RM: green wall", [])
        hgraph.current_node = node7
        hgraph.addNode(node7)
        hgraph.addNode(StateNode(8, "R: green wall", []))
        hgraph.addEdge(Edge("id", 1, 2, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 5, 3, "MPPI", "controller"))
        hgraph.addEdge(Edge("id", 2, 4, "Forward Model", "controller", True))
        hgraph.addEdge(Edge("id", 4, 8, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 8, 7, "model fitting", "controller", True))
        hgraph.addEdge(Edge("id", 6, 5, "MPC", "controller"))
        hgraph.addNode(StateNode(11, "R: duck", []))
        hgraph.addNode(StateNode(12, "RM: duck", []))
        hgraph.addNode(StateNode(13, "target duck", []))
        hgraph.addNode(StateNode(14, "RM: cube", []))
        hgraph.addEdge(Edge("id", 7, 11, "MPC", "controller"))
        hgraph.addEdge(Edge("id", 11, 12, "LSTM", "controller"))
        hgraph.addEdge(Edge("id", 12, 13, "RMPPI", "controller"))
        hgraph.addEdge(Edge("id", 13, 14, "MPC", "controller"))
        hgraph.addEdge(Edge("id", 14, 3, "EMPPI", "controller"))

    if stage==7:
        node4 = ObjectNode(4, "RM: cube", [])
        hgraph.addNode(node4)
        hgraph.addNode(StateNode(5, "RM: cube", []))
        hgraph.addNode(StateNode(6, "(aborted) R: target green wall", []))
        node7 = ObjectNode(7, "RM: green wall", [])
        hgraph.addNode(node7)
        hgraph.addNode(StateNode(8, "R: green wall", []))
        hgraph.addEdge(Edge("id", 1, 2, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 5, 3, "MPPI", "controller"))
        hgraph.addEdge(Edge("id", 2, 4, "Forward Model", "controller", True))
        hgraph.addEdge(Edge("id", 4, 8, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 8, 7, "model fitting", "controller", True))
        hgraph.addEdge(Edge("id", 6, 5, "MPC", "controller"))
        hgraph.addNode(StateNode(11, "R: duck", []))
        hgraph.addNode(StateNode(12, "RM: duck", []))
        node13 = StateNode(13, "target duck", [])
        hgraph.current_node = node13
        hgraph.addNode(node13)
        hgraph.addNode(StateNode(14, "RM: cube", []))
        hgraph.addEdge(Edge("id", 7, 11, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 11, 12, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 12, 13, "RMPPI", "controller", True))
        hgraph.addEdge(Edge("id", 13, 14, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 14, 3, "EMPPI", "controller"))

    hgraph.visualise()

if __name__ == "__main__":
    # 8 stages of the hgraph, in
    # main(7)
    for i in range(8):
        main(i)

