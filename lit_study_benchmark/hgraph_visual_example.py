from robot_brain.graph.HGraph import HGraph
from robot_brain.graph.Node import Node
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode
from robot_brain.graph.Edge import Edge

def main(stage):
    # stage indicates the stage the hgraph is at 
    # create a hypothesis graph and visualise

    hgraph = HGraph()

    if stage==0:
        # adding start and target node
        node1 = ObjectSetNode(1, "start robot and box", [])
        hgraph.addStartNode(node1)
        node2 = ConfSetNode(2, "target box", [])
        hgraph.addTargetNode(node2)
    
    if stage>=1:
        # adding expanded start and target node
        node1 = ObjectSetNode(1, "start robot", [])
        hgraph.addStartNode(node1)
        node2 = ObjectSetNode(2, "start box", [])
        hgraph.addStartNode(node2)
        node3 = ConfSetNode(3, "target box", [])
        hgraph.addTargetNode(node3)
        
    if stage>=2:
        node4 = ConfSetNode(4, "robot to box", [])
        hgraph.addNode(node4)
        edge1 = Edge("id", 2, 3, "EMPPI", "controller")
        hgraph.addEdge(edge1)
        edge2 = Edge("id", 4, 2, "warmup stage", "controller")
        edge2.path = True
        hgraph.addEdge(edge2)
 
    if stage>=3:
        edge3 = Edge("id", 5, 4, "MPC", "controller")
        hgraph.addEdge(edge3)

        node5 = ConfSetNode(5, "robot with model", [])
        hgraph.addNode(node5)

        edge4 = Edge("id", 1, 5, "PEM", "controller")
        edge4.path = True
        hgraph.addEdge(edge4)

    if stage==4:
        hgraph.current_node = node1

    if stage>=5:
        hgraph.current_node = node5
        edge3.path = True
    
    if stage>=6:
        hgraph.current_node = node4
        edge3.path = True
        edge2.path = True
    
    if stage>=7:
        hgraph.current_node = node2
        edge1.path = True
  
    if stage==8:
        hgraph.current_node = node3
 


    hgraph.visualise()

if __name__ == "__main__":
    # 8 stages of the hgraph, in
    main(1)

