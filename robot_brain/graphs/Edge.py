
class Edge:

    def __init__(self, source, to, verb, controller):
        self.source = source
        self.to = to
        self.conf_set_tail = None
        self.conf_set_head = None
        self.verb = verb
        self.controller = controller
        self.d = "inseart dynamical model please"
        self.alpha = None
