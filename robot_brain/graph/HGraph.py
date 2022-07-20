# import pandas as pd
from robot_brain.graph.Graph import Graph
from robot_brain.graph.Node import Node
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode 
from pyvis.network import Network
import numpy as np
import pyarrow.feather as feather
from robot_brain.global_variables import *

class HGraph(Graph):

    def __init__(self):
        Graph.__init__(self)
        self.is_class = "hgraph"
        self.target_node = None
        self.start_node = None
        self.current_node = None

    def addNode(self, node):
        if isinstance(node, ChangeOfConfSetNode):
            raise TypeError("ChangeOfConfSetNode's are not allowed in HGraph")

        self.nodes.append(node)

    def addTargetNode(self, node):
        if not isinstance(node, ConfSetNode):
            raise TypeError("Only ConfSetNode is allowed as targetNode")
         
        self.addNode(node)  
        self.target_node = node

    def visualiseHGraph(self):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """
        net = Network(directed=True)

        net.set_edge_smooth('dynamic')
        # add nodes
        for node in self.nodes:
            group = node.__class__.__name__
            label = " "
            x = 5.0
            y = 5.0
            color= {
                    'color':'#ffffa3',
                    'highlight':'red',
                    'hover': 'blue',
             } 

            if node.type is not None:
                group = node.type
                label = node.type
                color = 'blue'
                if node.type is TARGET_NODE:
                    x = 10.0
                    y = 5.0
                elif node.type is STARTING_NODE:
                    x = 1.0
                    y = 5.0

            # print("node: {}. class {}".format(node.type, group))

            net.add_node(node.id,
                    title=group + ":<br>" + node.toString() + "<br>",
                    color=color,
                    x=x,
                    y=y, 
                    label=label,
                    group=group)

        # add edges
        for edge in self.edges:

            dashes = False
            if edge.path == False:

                dashes = True

            net.add_edge(edge.source,
                    edge.to,
                    weight=1.0,
                    dashes=dashes,
                    group=edge.verb,
                    label=edge.verb,
                    title="edge:<br>" + edge.toString() + "<br>",
                    )
        
        # if you want to edit cusomize the graph
        net.show_buttons(filter_=['physics'])

        net.show("delete.html")



    @property
    def target_node(self):
        return self._target_node

    @target_node.setter
    def target_node(self, val):
        # input sanitization
        self._target_node = val
