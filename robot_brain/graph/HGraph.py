# import pandas as pd
from robot_brain.graph.Graph import Graph
from robot_brain.graph.Node import Node
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode 
from pyvis.network import Network
import os
from robot_brain.global_variables import *


class HGraph(Graph):

    def __init__(self):
        Graph.__init__(self)
        self.target_nodes = []
        self.start_nodes = []
        self.current_node = None

    def visualise(self, path=None):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """
        net = Network(bgcolor=FIG_BG_COLOR, height="450px", directed=True)
        
        # set a custom style sheet
        net.path = os.getcwd() + "/../robot_brain/dashboard/assets/graph_template.html"


        net.set_edge_smooth('dynamic')


        for node in self.start_nodes:
            if node == self.current_node: 
                continue
            net.add_node(node.id,
                    title = "Starting Node:<br>" + node.toString() + "<br>",
                    x=1.0,
                    y=1.0,
                    label = node.name,
                    borderWidth= 1,
                    borderWidthSelected= 2,
                    color= {
                        'border': '#2B7CE9', # blue
                        'background': '#97C2FC',
                        'highlight': {
                            'border': '#2B7CE9',
                            'background': '#D2E5FF'
                            }
                        },
                    group = "start_nodes")

        for node in self.target_nodes:
            if node == self.current_node: 
                continue
            net.add_node(node.id,
                    title = "Target Node:<br>" + node.toString() + "<br>",
                    x=90.0,
                    y=90.0,
                    label = node.name,
                    color= {
                        'border': '#009900', # green
                        'background': '#00ff00',
                        'highlight': {
                            'border': '#009900',
                            'background': '#99ff99'
                            }
                        },
                    group = "target_nodes")

        for node in self.nodes:
            if node == self.current_node: 
                continue 
            net.add_node(node.id,
                    title = "Node:<br>" + node.toString() + "<br>",
                    x=1.0,
                    y=1.0,
                    color= {
                        'border': '#ffa500', # yellow
                            'background': '#ffff00',
                        'highlight': {
                            'border': '#ffa500',
                            'background': '#ffff99'
                            }
                        },
                    label = " ",
                    group = node.__class__.__name__)

        if self.current_node is not None:
            net.add_node(self.current_node.id,
                    title = "Current node:<br>" + self.current_node.toString() + "<br>",
                    x=1.0,
                    y=1.0,
                    color= {
                        'border': '#fb4b50', # red 
                        'background': '#fb7e81',
                        'highlight': {
                            'border': '#fb4b50',
                            'background': '#fcbcc4'
                            }
                        },label = self.current_node.name,
                    group = "current_node")


        # add edges
        for edge in self.edges:

            dashes = False
            if edge.path == False:
                dashes = True

            net.add_edge(edge.source,
                    edge.to,
                    weight=1.0,
                    dashes=dashes,
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
        if isinstance(node, ChangeOfConfSetNode):
            raise TypeError("ChangeOfConfSetNode's are not allowed in HGraph")

        self.nodes.append(node)

    def addStartNode(self, node):
        if not isinstance(node, ObjectSetNode):
            raise TypeError("ObjectSetNode's are only allowed as starting node in HGraph")

        self.start_nodes.append(node)


    def addTargetNode(self, node):
        if not isinstance(node, ConfSetNode):
            raise TypeError("ConfSetNode's are only allowed as target node in HGraph")

        self.target_nodes.append(node)


