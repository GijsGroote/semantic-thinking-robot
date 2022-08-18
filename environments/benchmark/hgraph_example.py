from robot_brain.graph.HGraph import HGraph
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.Edge import Edge

hgraph = HGraph()

def main(stage):

    hgraph = HGraph()
    node1 = ObjectSetNode(1, "start robot", [])
    hgraph.addStartNode(node1)
    node4 = ConfSetNode(3, "target sphere", [])
    hgraph.addTargetNode(node4)

    if stage<1:
        hgraph.addStartNode(ObjectSetNode(2, "start sphere", []))
    else:
        node2 = ObjectSetNode(2, "R: start sphere", [])
        hgraph.addStartNode(node2)

    if stage>=1:
        node3 = ConfSetNode(4, "RM: sphere", [])
        hgraph.addNode(node3)
        hgraph.addEdge(Edge("id", 2, 4, "warmup stage", "controller", True))
        if stage<=4:
            hgraph.addEdge(Edge("id", 4, 3, "EMPPI", "controller"))
        else:

            hgraph.addEdge(Edge("id", 4, 3, "EMPPI", "controller", True))

    if stage>=2 and stage<4:
        hgraph.addNode(ConfSetNode(5, "M: robot", []))
        hgraph.addStartNode(ObjectSetNode(2, "R: start sphere", []))
        hgraph.addEdge(Edge("id", 1, 5, "IPEM", "controller", True))
        hgraph.addEdge(Edge("id", 5, 2, "MPC", "controller"))
        if stage==3:
            hgraph.current_node = node1

    if stage==4 or stage==5 :
        hgraph.addEdge(Edge("id", 5, 2, "MPC", "controller", True))
        hgraph.addNode(ConfSetNode(5, "M: robot", []))
        hgraph.addStartNode(ObjectSetNode(2, "R: start sphere", []))
        hgraph.addEdge(Edge("id", 1, 5, "IPEM", "controller", True))
 

        if stage ==4:
            hgraph.current_node = node3
        elif stage ==5:

            hgraph.current_node = node4
    hgraph.visualise()

if __name__ == "__main__":
    # 8 stages of the hgraph, in
    for i in range(6):
        main(i)
    # main(5)
