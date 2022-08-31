from robot_brain.graph.h_graph import HGraph
from robot_brain.graph.conf_set_node import ConfSetNode
from robot_brain.graph.object_set_node import ObjectSetNode
from robot_brain.graph.edge import Edge

hgraph = HGraph()

def main(stage):

    hgraph = HGraph()
    node1 = ObjectSetNode(1, "start robot", [])
    hgraph.add_start_node(node1)
    node4 = ConfSetNode(3, "target sphere", [])
    hgraph.add_target_node(node4)

    if stage<1:
        hgraph.add_start_node(ObjectSetNode(2, "start sphere", []))
    else:
        node2 = ObjectSetNode(2, "R: start sphere", [])
        hgraph.add_start_node(node2)

    if stage>=1:
        node3 = ConfSetNode(4, "RM: sphere", [])
        hgraph.add_node(node3)
        hgraph.add_edge(Edge("id", 2, 4, "warmup stage", "controller", True))
        if stage<=4:
            hgraph.add_edge(Edge("id", 4, 3, "EMPPI", "controller"))
        else:

            hgraph.add_edge(Edge("id", 4, 3, "EMPPI", "controller", True))

    if stage>=2 and stage<4:
        hgraph.add_node(ConfSetNode(5, "M: robot", []))
        hgraph.add_start_node(ObjectSetNode(2, "R: start sphere", []))
        hgraph.add_edge(Edge("id", 1, 5, "IPEM", "controller", True))
        hgraph.add_edge(Edge("id", 5, 2, "MPC", "controller"))
        if stage==3:
            hgraph.current_node = node1

    if stage>=4:
        hgraph.add_edge(Edge("id", 5, 2, "MPC", "controller", True))
        node5 = ConfSetNode(5, "M: robot", [])
        hgraph.add_node(node5)
        hgraph.add_edge(Edge("id", 1, 5, "IPEM", "controller", True))
        if stage ==4:
            hgraph.current_node = node5
        elif stage ==5:
            hgraph.current_node = node2

        elif stage ==6:
            hgraph.current_node = node3
        elif stage ==7:

            hgraph.current_node = node4
    hgraph.visualise()

if __name__ == "__main__":
    # 8 stages of the hgraph, in
    for i in range(4, 8):
        main(i)
    # main(5)
