

class SystemModel:
    """
    System model.
    """
    def __init__(self, model):
        # TODO: set a unique id such that the system model is unique
        self.name = None
        self.iden = None
        self.model = model

    def to_string(self) -> str:
        """ return a human readible format of an SystemModel object."""
        return "todo"

    @property
    def model(self):
        return self._model

    @model.setter
    def model(self, val):
        assert (callable(val)), f"system model must be a callable function and is {type(val)}"
        self._model = val

def emptyfunction():
    pass

EMPTY_SYSTEM_MODEL = SystemModel(emptyfunction)
