import numpy as np
from robot_brain.global_planning.edge import Edge
from robot_brain.global_planning.identification_edge import IdentificationEdge

class PushIdentificationEdge(IdentificationEdge):

    def __init__(self, iden, source, to, verb, controller, path=False):

        Edge.__init__(self, iden, source, to, verb, controller, path)
        print("creaetd an pushidentificatiedge")

