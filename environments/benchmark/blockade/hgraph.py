from robot_brain.global_planning.hgraph.hgraph import HGraph
from robot_brain.global_planning.conf_set_node import ConfSetNode
from robot_brain.global_planning.object_set_node import ObjectSetNode
from robot_brain.global_planning.edge import Edge

def main(stage):

    hgraph = HGraph()

    hgraph.addStartNode(ObjectSetNode(1, "start robot", []))
    hgraph.addTargetNode(ConfSetNode(3, "target cube", []))
    hgraph.addStartNode(ObjectSetNode(2, "R: start cube", []))

    if stage==1:

        node1 = ObjectSetNode(1, "start robot", [])
        hgraph.current_node = node1
        hgraph.addStartNode(node1)
        hgraph.addNode(ObjectSetNode(4, "RM: cube", []))
        hgraph.addEdge(Edge("id", 1, 2, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 4, 3, "MPPI", "controller"))
        hgraph.addEdge(Edge("id", 2, 4, "Forward Model", "controller", True))

    if stage==2:
        
        node4 = ObjectSetNode(4, "RM: cube", [])
        hgraph.current_node = node4
        hgraph.addNode(node4)
        hgraph.addEdge(Edge("id", 1, 2, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 4, 3, "MPPI", "controller"))
        hgraph.addEdge(Edge("id", 2, 4, "Forward Model", "controller", True))
 

    if stage>=3 and stage<5:

        node4 = ObjectSetNode(4, "RM: cube", [])
        if stage == 3:
            hgraph.current_node = node4
        hgraph.addNode(node4)

        hgraph.addNode(ConfSetNode(5, "RM: cube", []))
        hgraph.addNode(ConfSetNode(6, "R: target green wall", []))
        node7 = ConfSetNode(7, "RM: green wall", [])
        hgraph.addNode(node7)
        hgraph.addNode(ConfSetNode(8, "R: green wall", []))
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
        node4 = ObjectSetNode(4, "RM: cube", [])
        hgraph.addNode(node4)
        hgraph.addNode(ConfSetNode(5, "RM: cube", []))
        hgraph.addNode(ConfSetNode(6, "(aborted) R: target green wall ", []))
        node7 = ObjectSetNode(7, "RM: green wall", [])
        hgraph.current_node = node7
        hgraph.addNode(node7)
        hgraph.addNode(ConfSetNode(8, "R: green wall", []))
        hgraph.addEdge(Edge("id", 1, 2, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 5, 3, "MPPI", "controller"))
        hgraph.addEdge(Edge("id", 2, 4, "Forward Model", "controller", True))
        hgraph.addEdge(Edge("id", 4, 8, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 8, 7, "model fitting", "controller", True))
        hgraph.addEdge(Edge("id", 6, 5, "MPC", "controller"))
        hgraph.addEdge(Edge("id", 7, 3, "MPPI", "controller"))

    if stage==6:
        node4 = ObjectSetNode(4, "RM: cube", [])
        hgraph.addNode(node4)
        hgraph.addNode(ConfSetNode(5, "RM: cube", []))
        hgraph.addNode(ConfSetNode(6, "(aborted) R: target green wall", []))
        node7 = ObjectSetNode(7, "RM: green wall", [])
        hgraph.current_node = node7
        hgraph.addNode(node7)
        hgraph.addNode(ConfSetNode(8, "R: green wall", []))
        hgraph.addEdge(Edge("id", 1, 2, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 5, 3, "MPPI", "controller"))
        hgraph.addEdge(Edge("id", 2, 4, "Forward Model", "controller", True))
        hgraph.addEdge(Edge("id", 4, 8, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 8, 7, "model fitting", "controller", True))
        hgraph.addEdge(Edge("id", 6, 5, "MPC", "controller"))
        hgraph.addNode(ConfSetNode(11, "R: duck", []))
        hgraph.addNode(ConfSetNode(12, "RM: duck", []))
        hgraph.addNode(ConfSetNode(13, "target duck", []))
        hgraph.addNode(ConfSetNode(14, "RM: cube", [])) 
        hgraph.addEdge(Edge("id", 7, 11, "MPC", "controller"))
        hgraph.addEdge(Edge("id", 11, 12, "LSTM", "controller"))
        hgraph.addEdge(Edge("id", 12, 13, "RMPPI", "controller"))
        hgraph.addEdge(Edge("id", 13, 14, "MPC", "controller"))
        hgraph.addEdge(Edge("id", 14, 3, "EMPPI", "controller"))
 
    if stage==7:
        node4 = ObjectSetNode(4, "RM: cube", [])
        hgraph.addNode(node4)
        hgraph.addNode(ConfSetNode(5, "RM: cube", []))
        hgraph.addNode(ConfSetNode(6, "(aborted) R: target green wall", []))
        node7 = ObjectSetNode(7, "RM: green wall", [])
        hgraph.addNode(node7)
        hgraph.addNode(ConfSetNode(8, "R: green wall", []))
        hgraph.addEdge(Edge("id", 1, 2, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 5, 3, "MPPI", "controller"))
        hgraph.addEdge(Edge("id", 2, 4, "Forward Model", "controller", True))
        hgraph.addEdge(Edge("id", 4, 8, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 8, 7, "model fitting", "controller", True))
        hgraph.addEdge(Edge("id", 6, 5, "MPC", "controller"))
        hgraph.addNode(ConfSetNode(11, "R: duck", []))
        hgraph.addNode(ConfSetNode(12, "RM: duck", []))
        node13 = ConfSetNode(13, "target duck", [])
        hgraph.current_node = node13
        hgraph.addNode(node13)
        hgraph.addNode(ConfSetNode(14, "RM: cube", [])) 
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

