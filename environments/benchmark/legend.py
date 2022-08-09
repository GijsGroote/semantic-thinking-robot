from robot_brain.graph.HGraph import HGraph
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.graph.Edge import Edge

def main():

    hgraph = HGraph()

    hgraph.addStartNode(ObjectSetNode(1, "start node", []))
    hgraph.addNode(ObjectSetNode(4, "generated node", []))
    hgraph.addTargetNode(ConfSetNode(3, "target node", []))


    node2 = ObjectSetNode(2, "curent node", [])
    hgraph.current_node = node2
    hgraph.addStartNode(node2)
    hgraph.addEdge(Edge("id", 4, 3, "transition, no planning completed", "controller"))
    hgraph.addEdge(Edge("id", 2, 4, "transition, no planning completed", "controller"))
    hgraph.addEdge(Edge("id", 1, 2, "transition, motion planning completed", "controller", True))

    
    hgraph.visualise()

if __name__ == "__main__":
    # 8 stages of the hgraph, in
    main()

