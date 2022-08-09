from robot_brain.graph.HGraph import HGraph
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.Edge import Edge


hgraph = HGraph()

def main(stage):
    
    hgraph = HGraph()
    node4 = ConfSetNode(4, "target duck", [])
    hgraph.addTargetNode(node4)
    hgraph.addTargetNode(ConfSetNode(5, "target cube", []))
    hgraph.addStartNode(ObjectSetNode(1, "start robot", []))

    if stage==0:
        hgraph.addStartNode(ObjectSetNode(2, "start duck", []))
        hgraph.addStartNode(ObjectSetNode(3, "start cube", []))

    if stage >=1:
        hgraph.addStartNode(ObjectSetNode(1, "start robot", []))
        hgraph.addStartNode(ObjectSetNode(2, "R: start duck", []))
        if stage<5:
            hgraph.addStartNode(ObjectSetNode(3, "start cube", []))
        hgraph.addTargetNode(ConfSetNode(5, "target cube", []))
        hgraph.addNode(ConfSetNode(6, "M: robot", []))
        node7 = ConfSetNode(7, "RM: duck", [])
        hgraph.addNode(node7)
        mpc_planned = False
        if stage==2:
            hgraph.current_node = node7
        if stage>1:
            mpc_planned = True
        hgraph.addEdge(Edge("id", 1, 6, "PEM", "controller", True))
        hgraph.addEdge(Edge("id", 6, 2, "MPC", "controller", mpc_planned))
        hgraph.addEdge(Edge("id", 2, 7, "model fitting", "controller", True))
        if stage<3:
            hgraph.addEdge(Edge("id", 7, 4, "MPC", "controller"))

    if stage==3 or stage==4:
        hgraph.addNode(ConfSetNode(8, "R: cube", []))
        hgraph.addNode(ConfSetNode(9, "RM: cube", []))
        hgraph.addNode(ConfSetNode(10, "target cube", []))
        hgraph.addEdge(Edge("id", 8, 9, "forward model", "controller", True))
        if stage==3:
            hgraph.current_node = node7
            hgraph.addEdge(Edge("id", 9, 10, "MPPI", "controller"))
            hgraph.addEdge(Edge("id", 7, 8, "MPC", "controller"))
        hgraph.addNode(ConfSetNode(11, "RM: duck", []))
        planned = False
        if stage==4:
            hgraph.current_node = node4
            hgraph.addEdge(Edge("id", 9, 10, "MPPI", "controller", True))
            hgraph.addEdge(Edge("id", 7, 8, "MPC", "controller", True))
            planned = True
        hgraph.addEdge(Edge("id", 10, 11, "MPC", "controller", planned))
        hgraph.addEdge(Edge("id", 11, 4, "MPC", "controller", planned))

    if stage==5:
        node3 = ObjectSetNode(3, "RM: start cube", [])
        hgraph.addStartNode(node3)
        hgraph.addNode(ConfSetNode(8, "R: cube", []))
        hgraph.addNode(ConfSetNode(9, "RM: cube", []))
        hgraph.addNode(ConfSetNode(10, "target cube", []))
        hgraph.addEdge(Edge("id", 8, 9, "forward model", "controller", True))
        hgraph.addNode(ConfSetNode(11, "RM: duck", []))
        hgraph.addEdge(Edge("id", 10, 11, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 11, 4, "MPC", "controller", True))
        hgraph.current_node = node3
        hgraph.addEdge(Edge("id", 9, 10, "MPPI", "controller", True))
        hgraph.addEdge(Edge("id", 7, 8, "MPC", "controller", True))
        hgraph.current_node 
        hgraph.addEdge(Edge("id", 4, 3, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 3, 5, "MPPI", "controller"))

    hgraph.visualise()

if __name__ == "__main__":
    # 8 stages of the hgraph, in
    # for i in range(6):
    #     main(i)
    main(5) 
