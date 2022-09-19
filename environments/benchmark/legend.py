from robot_brain.graph.h_graph import HGraph
from robot_brain.graph.conf_set_node import ConfSetNode
from robot_brain.graph.object_set_node import ObjectSetNode
from robot_brain.graph.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.graph.edge import Edge

def main():

    hgraph = HGraph()

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

