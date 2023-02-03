import os
from pyvis.network import Network
from robot_brain.global_planning.graph import Graph
from robot_brain.global_planning.node import Node
from robot_brain.global_planning.obstacle_node import ObstacleNode
from robot_brain.global_planning.change_of_state_node import ChangeOfStateNode
from robot_brain.global_variables import FIG_BG_COLOR


class KGraph(Graph):
    """
    Knowledge graph.
    """
    def __init__(self):
        Graph.__init__(self)

# T5HIS IS SOME STUFF TO INITIALISE THE kgRAPH LATER
##  temp KGraph
            # kgraph = KGraph()
            # # the robot
            # node1 = ObjectSetNode(1, "robot", [])
            # kgraph.add_node(node1)
            # node2 = ChangeOfConfSetNode(2, "position", [])
            # kgraph.add_node(node2)
            # kgraph.add_edge(Edge("id", 1, 2, "MPC", "PEM"))
            #
            # self.kgraph = kgraph
            # self.kgraph.visualise(
            #     path="/home/gijs/Documents/semantic-thinking-robot/dashboard/data/knowledge_graph.html"
            
    def visualise(self, path=None):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """
        # net = Network(bgcolor=FIG_BG_COLOR, height="450px", directed=True)
        net = Network(height="450px", directed=True)

        # set a custom style sheet
        # TODO: relative path (which works)
        # net.path = os.getcwd() + 
        net.path = "/home/gijs/Documents/semantic-thinking-robot/dashboard/assets/graph_template.html"


        net.set_edge_smooth('dynamic')
        for node in self.nodes:
            if isinstance(node, ObstacleNode):
                # TODO: relative path, somehow this image shows, but not if it runs on the server...
                # os.getcwd() + "/../lit_study_benchmark/" +  node.name + ".png"
                path_to_png = "/home/gijs/Documents/semantic-thinking-robot"\
                        "/dashboard/assets/images/" +  node.name + ".png"

                if os.path.exists(path_to_png):
                    net.add_node(node.iden,
                            title = "Node:<br>" + node.to_string() + "<br>",
                            x=1.0,
                            y=1.0,
                            color= {
                                'border': '#000000', # grey and black
                                'background': '#808080',
                                'highlight': {
                                    'border': '#000000',
                                    'background': '#a6a6a6'
                                    }
                                },
                            image= path_to_png,
                            shape= "circularImage",
                            label = " ",
                            group = node.__class__.__name__
                            )


            if isinstance(node, ChangeOfStateNode):
                net.add_node(node.iden,
                        title = "Node:<br>" + node.to_string() + "<br>",
                        x=1.0,
                        y=1.0,
                        color= {
                            'border': '#2B7CE9', # blue
                            'background': '#97C2FC',
                            'highlight': {
                                'border': '#2B7CE9',
                                'background': '#D2E5FF'
                                }
                            },
                        label = " ",
                        group = node.__class__.__name__
                        )


        # add edges
        for edge in self.edges:

            net.add_edge(edge.source,
                    edge.to,
                    weight=1.0,
                    label=edge.verb,
                    title="edge:<br>" + edge.to_string() + "<br>",
                    )

        # if you want to edit cusomize the graph
        # net.show_buttons(filter_=['physics'])

        if path is None:
            net.show("delete.html")
        else:
            net.write_html(path)


    def add_node(self, node):
        if not(isinstance(node, Node)):
            raise TypeError("todo: only allowed in KGraph")
        self.nodes.append(node)
