class RunnoutOfControlMethodsException(Exception):
    """ All possible combinations of controllers and models are on the blacklist. """

class NoPathExistsException(Exception):
    """ No valid path exists from start to target state. """

class PlanningTimeElapsedException(Exception):
    """ Computation time has exceeded allowed time. """

class FaultDetectedException(Exception):
    """ A fault is detected during execution. """

