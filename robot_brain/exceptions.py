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

class MovableObjectDetectedException(Exception):
    """ A movable object was detected pushed. """

class PushAnUnmovableObjectException(Exception):
    """ A unmovable object was insuccesfully pushed. """

class LoopDetectedException(Exception):
    """ A loop was detected in the HGraph."""

class TwoEdgesPointToSameNodeException(Exception):
    """ Two non-failed edges point to the same node in HGraph. """
