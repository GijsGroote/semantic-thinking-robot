class Edge:
    """
    Edge or transition, describes the way/method of transitioning from
    one Node to another Node.
    """

    def __init__(self, id, source, to, verb, controller):
        self.id = id
        self.source = source
        self.to = to
        self.verb = verb
        self.controller = controller
        self.d = "use a actual dynamical model please"
        self.alpha = None

    def toString(self):
        """
        Creates readable format of an Edge
        :return: String
        """
        return "id: {}\nverb: {}".format(self.id, self.verb)

    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, val):
        self._id = val

    @property
    def source(self):
        return self._source

    @source.setter
    def source(self, val):
        self._source = val

    @property
    def to(self):
        return self._to

    @to.setter
    def to(self, val):
        self._to = val

    @property
    def verb(self):
        return self._verb

    @verb.setter
    def verb(self, val):
        self._verb = val

    @property
    def controller(self):
        return self._controller

    @controller.setter
    def controller(self, contr):
        # todo: check contr is a Controller object
        self._controller = contr
