from robot_brain.graphs.HGraph import HGraph
from robot_brain.graphs.ObjectSetNode import ObjectSetNode
from robot_brain.graphs.Edge import Edge

def main():

    # create hypothesis graph
    hgraph = HGraph()

    target_node = ObjectSetNode(None, 3)
    middle_node = ObjectSetNode(None, 2)
    current_node = ObjectSetNode(None, 1)

    hgraph.addTargetNode(target_node)
    hgraph.addNode(current_node)
    hgraph.addNode(middle_node)

    edge1 = Edge(2, 3, "push")
    edge2 = Edge(1, 2, "drive mpc")
    edge3 = Edge(1, 2, "drive pid")

    hgraph.addEdge(edge1)
    hgraph.addEdge(edge2)
    hgraph.addEdge(edge3)

    hgraph.visualise()




if __name__ == '__main__':
    main()
