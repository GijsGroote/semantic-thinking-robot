class RunnoutOfControlMethodsException(Exception):
    """ All possible combinations of controllers and models are on the blacklist. """

class NoPathExistsException(Exception):
    """ No valid path exists from start to target state. """

class PlanningTimeElapsedException(Exception):
    """ Computation time has exceeded allowed time. """

class FaultDetectedException(Exception):
    """ A fault is detected during execution. """

class NoBestPushPositionException(Exception):
    """ No best push position around an object could be found. """

class NoTargetPositionFoundException(Exception):
    """ No target position can be found to push object toward. """
