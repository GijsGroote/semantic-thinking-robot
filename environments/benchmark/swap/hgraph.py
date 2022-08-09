from robot_brain.graph.HGraph import HGraph
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.Edge import Edge

hgraph = HGraph()

def main(stage):

    hgraph = HGraph()
    hgraph.addStartNode(ObjectSetNode(1, "start robot", []))
    hgraph.addTargetNode(ConfSetNode(2, "target robot", []))

    if stage==1:
        node3 = ConfSetNode(3, "M: robot", [])
        hgraph.current_node = node3
        hgraph.addNode(node3)
        hgraph.addEdge(Edge("id", 1, 3, "IPEM", "controller", True))
        hgraph.addEdge(Edge("id", 3, 2, "MPC", "controller"))

    if stage==2 or stage==3:
        node3 = ConfSetNode(3, "M: robot", [])
        hgraph.current_node = node3
        hgraph.addNode(node3)
        hgraph.addEdge(Edge("id", 1, 3, "IPEM", "controller", True))
        node5 = ConfSetNode(5, "RM: red cube ", [])
        if stage ==3:
            hgraph.current_node = node5 
        hgraph.addNode(node5)
        hgraph.addNode(ConfSetNode(4, "R: red cube", []))
        hgraph.addNode(ConfSetNode(6, "RM: target red cube", []))
        hgraph.addEdge(Edge("id", 3, 4, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 4, 5, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 5, 6, "EMPPI", "controller"))
        hgraph.addEdge(Edge("id", 6, 2, "MPC", "controller"))

    if stage==4 or stage==5:
        hgraph.addNode(ConfSetNode(3, "M: robot", []))
        hgraph.addEdge(Edge("id", 1, 3, "IPEM", "controller", True))
        hgraph.addNode(ConfSetNode(4, "R: red cube", []))
        node5 = ConfSetNode(5, "RM: red cube ", [])
        hgraph.current_node = node5 
        hgraph.addNode(node5)
        hgraph.addNode(ConfSetNode(6, "(aborted) RM: target red cube", []))
        hgraph.addEdge(Edge("id", 3, 4, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 4, 5, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 6, 2, "MPC", "controller"))
        hgraph.addNode(ConfSetNode(7, "R: blue cube", []))
        node8 = ConfSetNode(8, "RM: blue cube ", []) 
        hgraph.addNode(node8)
        hgraph.addNode(ConfSetNode(9, "RM: target blue cube", []))
        if stage==5:
            hgraph.current_node = node8
        else:
            hgraph.current_node = node5
        hgraph.addEdge(Edge("id", 5, 7, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 7, 8, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 8, 9, "RMPPI", "controller"))
        hgraph.addEdge(Edge("id", 9, 2, "MPC", "controller"))

    if stage == 6:
        hgraph.addNode(ConfSetNode(3, "M: robot", []))
        hgraph.addEdge(Edge("id", 1, 3, "IPEM", "controller", True))
        hgraph.addNode(ConfSetNode(4, "R: red cube", []))
        hgraph.addNode(ConfSetNode(5, "RM: red cube ", []))
        hgraph.addNode(ConfSetNode(6, "(aborted) RM: target red cube", []))
        hgraph.addEdge(Edge("id", 3, 4, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 4, 5, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 6, 2, "MPC", "controller"))
        hgraph.addNode(ConfSetNode(7, "R: blue cube", []))
        hgraph.addNode(ConfSetNode(8, "RM: blue cube ", []) )
        hgraph.addNode(ConfSetNode(9, "(aborted) RM: target blue cube", []))
        hgraph.addEdge(Edge("id", 5, 7, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 7, 8, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 9, 2, "MPC", "controller"))
        hgraph.addNode(ConfSetNode(10, "R: green cube", []))
        node11 = ConfSetNode(11, "RM: green cube ", []) 
        hgraph.addNode(node11)
        node12 = ConfSetNode(12, "RM: target green cube", []) 
        hgraph.current_node = node12 
        hgraph.addNode(node12)
        hgraph.addEdge(Edge("id", 8, 10, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 10, 11, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 11, 12, "RMPPI", "controller", True))
        hgraph.addEdge(Edge("id", 12, 2, "MPC", "controller"))
    
    if stage == 7:
        hgraph.addNode(ConfSetNode(11, "RM: green cube ", []))
        hgraph.addNode(ConfSetNode(12, "RM: target green cube", []))
        hgraph.addEdge(Edge("id", 1, 11, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 11, 12, "RMPPI", "controller", True))
        hgraph.addEdge(Edge("id", 12, 2, "MPC", "controller", True))

    hgraph.visualise()

if __name__ == "__main__":
    # 8 stages of the hgraph, in
    # for i in range(8):
    #     main(i)
    main(5)
