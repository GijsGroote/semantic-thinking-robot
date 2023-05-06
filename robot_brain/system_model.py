
def emptyfunction():
    pass

class SystemModel:
    """
    System model.
    """
    def __init__(self, model, name=None):

        if name is None:
            self.model = emptyfunction
        else:
            self.model = model
        self.name = name

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
