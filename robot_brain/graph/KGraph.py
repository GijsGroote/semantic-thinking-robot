from robot_brain.graph.Graph import Graph 
from robot_brain.graph.Node import Node
from robot_brain.graph.ConfSetNode import ConfSetNode
from robot_brain.graph.ObjectSetNode import ObjectSetNode
from robot_brain.graph.ChangeOfConfSetNode import ChangeOfConfSetNode 
from robot_brain.graph.Graph import Graph
from pyvis.network import Network

class KGraph(Graph):

    def __init__(self):
        Graph.__init__(self)
        self.is_class = "kgraph"
        # print("ha a knowledge graphs yo")

    def visualiseHGraph(self):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """
        net = Network(directed=True)

        net.set_edge_smooth('dynamic')


        # for node in self.start_nodes:
        #     if node == self.current_node: 
        #         continue
        #     net.add_node(node.id,
        #             title = "Starting Node:<br>" + node.toString() + "<br>",
        #             x=1.0,
        #             y=1.0,
        #             label = node.name,
        #             borderWidth= 1,
        #             borderWidthSelected= 2,
        #             color= {
        #                 'border': '#2B7CE9', # blue
        #                 'background': '#97C2FC',
        #                 'highlight': {
        #                     'border': '#2B7CE9',
        #                     'background': '#D2E5FF'
        #                     }
        #                 },
        #             group = "start_nodes")
        #
        # for node in self.target_nodes:
        #     if node == self.current_node: 
        #         continue
        #     net.add_node(node.id,
        #             title = "Target Node:<br>" + node.toString() + "<br>",
        #             x=90.0,
        #             y=90.0,
        #             label = node.name,
        #             color= {
        #                 'border': '#009900', # green
        #                 'background': '#00ff00',
        #                 'highlight': {
        #                     'border': '#009900',
        #                     'background': '#99ff99'
        #                     }
        #                 },
        #             group = "target_nodes")

        for node in self.nodes:
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
        net.show_buttons(filter_=['physics'])

        net.show("delete.html")



    def addNode(self, node):
        self.nodes.append(node)
