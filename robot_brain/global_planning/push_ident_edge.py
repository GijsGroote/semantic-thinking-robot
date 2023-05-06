from robot_brain.global_planning.identification_edge import IdentificationEdge
from robot_brain.controller.controller import Controller

class PushIdentificationEdge(IdentificationEdge):
    """ Identifiy an system model for a push action. """

    def __init__(self,
            iden: int,
            source: int,
            to: int,
            verb: str,
            controller: Controller,
            model_for_edge_iden: int,
            sys_model_name: str):

        # TODO: the controller here makes no sense, remove it.
        IdentificationEdge.__init__(self, iden, source, to, verb, controller, model_for_edge_iden, sys_model_name)
