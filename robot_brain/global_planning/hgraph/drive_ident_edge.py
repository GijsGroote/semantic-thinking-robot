from robot_brain.global_planning.hgraph.identification_edge import IdentificationEdge
from robot_brain.controller.controller import Controller

class DriveIdentificationEdge(IdentificationEdge):
    """ Collect data and generate a driving system model. """

    def __init__(self,
            iden: int,
            source: int,
            to: int,
            verb: str,
            controller: Controller,
            model_for_edge_iden: int, 
            sys_model_name: str):

        IdentificationEdge.__init__(self, iden, source, to, verb, controller, model_for_edge_iden, sys_model_name)
