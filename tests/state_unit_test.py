import pytest
import numpy as np

from robot_brain.state import State

def test_input_types():
    """ test if the input is not a numpy array should throw an exception """
    # State(pos=np.array(31))

    with pytest.raises(IndexError):
        State(pos=np.array(31))
    with pytest.raises(AttributeError):
        State(vel=True)
    with pytest.raises(AttributeError):
        State(acc=13)
    with pytest.raises(AttributeError):
        State(ang_p=13 / 70)
    with pytest.raises(AttributeError):
        State(ang_p=False)
    with pytest.raises(AttributeError):
        State(ang_p="a string")

    # self.assertTrue("input should be an numpy array" in str(context.exception))

def test_warning():
    """ test if warning is thrown for position of shape (2, ) """

    with pytest.warns(Warning):
        State(pos=np.array([3, 2]))
    with pytest.warns(Warning):
        State(vel=np.array([3, 2]))
    with pytest.warns(Warning):
        State(acc=np.array([3, 2]))


def test_exception():
    """ test if initializing the position with incorrect shape raises an exception """

    with pytest.raises(Exception):
        State(vel=np.array([31]))
    with pytest.raises(Exception):
        State(acc=np.array([31, -30, 50.0, 12]))
    with pytest.raises(Exception):
        State(ang_p=np.array([3, 1, 13, 15, 98]))
    with pytest.raises(Exception):
        State(pos=True)
    with pytest.raises(Exception):
        State(ang_v=np.array([3, 2, 5, 53]))
    with pytest.raises(Exception):
        State(ang_a=np.array([3, 52, True, 3]))
