import os
from pyvis.network import Network
from robot_brain.graph.graph import Graph
from robot_brain.graph.node import Node
from robot_brain.graph.conf_set_node import ConfSetNode
from robot_brain.graph.object_set_node import ObjectSetNode
from robot_brain.graph.change_of_conf_set_node import ChangeOfConfSetNode
from robot_brain.global_variables import FIG_BG_COLOR


class KGraph(Graph):
    """
    Knowledge graph.
    """
    def __init__(self):
        Graph.__init__(self)

    def visualise(self, path=None):
        """"
        Visualising is for testing, creating the plot in the dashboard is in dashboard/figures
        """
        net = Network(bgcolor=FIG_BG_COLOR, height="450px", directed=True)

        # set a custom style sheet
        # TODO: relative path (which works)
        # net.path = os.getcwd() + 
        net.path = "/home/gijs/Documents/semantic-thinking-robot/robot_brain/dashboard/assets/graph_template.html"


        net.set_edge_smooth('dynamic')
        for node in self.nodes:
            if isinstance(node, ObjectSetNode):
                # TODO: relative path, somehow this image shows, but not if it runs on the server...
                # os.getcwd() + "/../lit_study_benchmark/" +  node.name + ".png"
                path_to_png = "/home/gijs/Documents/semantic-thinking-robot"\
                        "/robot_brain/dashboard/assets/images/" +  node.name + ".png"

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


            if isinstance(node, ChangeOfConfSetNode):
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
        if not(isinstance(node, Node) and not isinstance(node, ConfSetNode)):
            raise TypeError("ObjectSetNodes's and ChangeOfConfSetNodes are only allowed in KGraph")
        self.nodes.append(node)
