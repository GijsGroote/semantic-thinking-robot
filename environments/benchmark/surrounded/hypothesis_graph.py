from robot_brain.global_planning.hgraph.hgraph import HGraph
from robot_brain.state_node import StateNode
from robot_brain.global_planning.object_node import ObjectNode
from robot_brain.global_planning.edge import Edge

hgraph = HGraph()

def main(stage):

    hgraph = HGraph()
    hgraph.addStartNode(ObjectNode(1, "start robot", []))
    hgraph.addTargetNode(StateNode(2, "target robot", []))

    if stage==1:
        node3 = StateNode(3, "M: robot", [])
        hgraph.current_node = node3
        hgraph.addNode(node3)
        hgraph.addEdge(Edge("id", 1, 3, "IPEM", "controller", True))
        hgraph.addEdge(Edge("id", 3, 2, "MPC", "controller"))

    if stage==2 or stage==3:
        node3 = StateNode(3, "M: robot", [])
        hgraph.current_node = node3
        hgraph.addNode(node3)
        hgraph.addEdge(Edge("id", 1, 3, "IPEM", "controller", True))
        node5 = StateNode(5, "RM: red cube ", [])
        if stage ==3:
            hgraph.current_node = node5
        hgraph.addNode(node5)
        hgraph.addNode(StateNode(4, "R: red cube", []))
        hgraph.addNode(StateNode(6, "RM: target red cube", []))
        hgraph.addEdge(Edge("id", 3, 4, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 4, 5, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 5, 6, "EMPPI", "controller"))
        hgraph.addEdge(Edge("id", 6, 2, "MPC", "controller"))

    if stage==4 or stage==5:
        hgraph.addNode(StateNode(3, "M: robot", []))
        hgraph.addEdge(Edge("id", 1, 3, "IPEM", "controller", True))
        hgraph.addNode(StateNode(4, "R: red cube", []))
        node5 = StateNode(5, "RM: red cube ", [])
        hgraph.current_node = node5
        hgraph.addNode(node5)
        hgraph.addNode(StateNode(6, "(aborted) RM: target red cube", []))
        hgraph.addEdge(Edge("id", 3, 4, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 4, 5, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 6, 2, "MPC", "controller"))
        hgraph.addNode(StateNode(7, "R: blue cube", []))
        node8 = StateNode(8, "RM: blue cube ", [])
        hgraph.addNode(node8)
        hgraph.addNode(StateNode(9, "RM: target blue cube", []))
        if stage==5:
            hgraph.current_node = node8
        else:
            hgraph.current_node = node5
        hgraph.addEdge(Edge("id", 5, 7, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 7, 8, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 8, 9, "RMPPI", "controller"))
        hgraph.addEdge(Edge("id", 9, 2, "MPC", "controller"))

    if stage == 6:
        hgraph.addNode(StateNode(3, "M: robot", []))
        hgraph.addEdge(Edge("id", 1, 3, "IPEM", "controller", True))
        hgraph.addNode(StateNode(4, "R: red cube", []))
        hgraph.addNode(StateNode(5, "RM: red cube ", []))
        hgraph.addNode(StateNode(6, "(aborted) RM: target red cube", []))
        hgraph.addEdge(Edge("id", 3, 4, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 4, 5, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 6, 2, "MPC", "controller"))
        hgraph.addNode(StateNode(7, "R: blue cube", []))
        hgraph.addNode(StateNode(8, "RM: blue cube ", []) )
        hgraph.addNode(StateNode(9, "(aborted) RM: target blue cube", []))
        hgraph.addEdge(Edge("id", 5, 7, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 7, 8, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 9, 2, "MPC", "controller"))
        hgraph.addNode(StateNode(10, "R: green cube", []))
        node11 = StateNode(11, "RM: green cube ", [])
        hgraph.addNode(node11)
        node12 = StateNode(12, "RM: target green cube", [])
        hgraph.current_node = node12
        hgraph.addNode(node12)
        hgraph.addEdge(Edge("id", 8, 10, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 10, 11, "LSTM", "controller", True))
        hgraph.addEdge(Edge("id", 11, 12, "RMPPI", "controller", True))
        hgraph.addEdge(Edge("id", 12, 2, "MPC", "controller"))

    if stage == 7:
        hgraph.addNode(StateNode(11, "RM: green cube ", []))
        hgraph.addNode(StateNode(12, "RM: target green cube", []))
        hgraph.addEdge(Edge("id", 1, 11, "MPC", "controller", True))
        hgraph.addEdge(Edge("id", 11, 12, "RMPPI", "controller", True))
        hgraph.addEdge(Edge("id", 12, 2, "MPC", "controller", True))

    hgraph.visualise()

if __name__ == "__main__":
    # 8 stages of the hgraph, in
    # for i in range(8):
    #     main(i)
    main(5)
