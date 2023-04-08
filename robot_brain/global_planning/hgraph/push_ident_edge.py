from robot_brain.global_planning.hgraph.identification_edge import IdentificationEdge
from robot_brain.controller.controller import Controller

class PushIdentificationEdge(IdentificationEdge):
    """ Identifiy an system model for a push action. """

    def __init__(self,
            iden: int,
            source: int,
            to: int,
            verb: str,
            controller: Controller,
            subtask_name: str,
            model_for_edge_iden: int):

        IdentificationEdge.__init__(self, iden, source, to, verb, controller, subtask_name, model_for_edge_iden)
