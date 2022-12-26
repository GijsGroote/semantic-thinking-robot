from robot_brain.global_planning.action_edge import ActionEdge

class PushActionEdge(ActionEdge):
    """ Push action edge controls all pushing actions. """

    def __init__(self, iden, source, to, verb, controller):
        ActionEdge.__init__(self, iden, source, to, verb, controller)


