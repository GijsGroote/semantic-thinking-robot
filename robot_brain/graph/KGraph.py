from robot_brain.graph.Graph import Graph 
from robot_brain.graph.Node import Node
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode 
from robot_brain.graph.Graph import Graph
from pyvis.network import Network
import os
from robot_brain.global_variables import *

class KGraph(Graph):

    def __init__(self):
        Graph.__init__(self)
        print("init")
        

    def visualise(self, path=None):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """
        # net = Network(bgcolor=FIG_BG_COLOR, height="450px", directed=True)
        
        net = Network(height="450px", directed=True)

        # set a custom style sheet
        net.path = os.getcwd() + "/../robot_brain/dashboard/assets/graph_template.html"


        net.set_edge_smooth('dynamic')
        for node in self.nodes:
            if isinstance(node, ObjectSetNode):
                
                image = None
                # use custom image if one exists
                #temp path, somehow this image shows, but not if it runs on the server...
                path_to_png = os.getcwd() + "/../lit_study_benchmark/" +  node.name + ".png"
                if os.path.exists(path_to_png):
                    image= path_to_png 
                print(os.path.exists(path_to_png))
                print(image)

                net.add_node(node.id,
                        title = "Node:<br>" + node.toString() + "<br>",
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
                        image= image, 
                        shape= "circularImage",
                        label = " ",
                        group = node.__class__.__name__
                        )


            if isinstance(node, ChangeOfConfSetNode):
                 net.add_node(node.id,
                        title = "Node:<br>" + node.toString() + "<br>",
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
                    title="edge:<br>" + edge.toString() + "<br>",
                    )

        # if you want to edit cusomize the graph
        # net.show_buttons(filter_=['physics'])

        if path is None:
            net.show("delete.html")
        else:
            net.write_html(path)


    def addNode(self, node): 
        if not(isinstance(node, Node) and not isinstance(node, ConfSetNode)):
            raise TypeError("ObjectSetNodes's and ChangeOfConfSetNodes are only allowed in KGraph")

        self.nodes.append(node)

