from robot_brain.global_planning.identification_edge import IdentificationEdge
from robot_brain.controller.controller import Controller

class PushIdentificationEdge(IdentificationEdge):

    def __init__(self,
            iden: int,
            source: int,
            to: int,
            verb: str,
            controller: Controller,
            model_for_edge_iden: int):

        IdentificationEdge.__init__(self, iden, source, to, verb, controller, model_for_edge_iden)

 
