import unittest
import numpy as np
from robot_brain.planning.State import State


class MyTestCase(unittest.TestCase):

    # todo: a test for quaternaions to euler angles for the ang_pos

    def test_input_types(self):
        """ test if the input is not a numpy array should throw an exception """
        with self.assertRaises(AssertionError) as context:
            State(pos=np.array(31))
            State(vel=True)
            State(acc=13)
            State(ang_p=13 / 70)
            State(ang_p=False)
            State(ang_p="a string")

        self.assertTrue("input should be an numpy array" in str(context.exception))

    def test_warning(self):
        """ test if warning is thrown for position of shape (2, ) """
        with self.assertWarns(Warning):
            State(pos=np.array([3, 2]))
        with self.assertWarns(Warning):
            State(vel=np.array([3, 2]))
        with self.assertWarns(Warning):
            State(acc=np.array([3, 2]))


    def test_exception(self):
        """ test if initializing the position with incorrect shape raises an exception """
        self.assertRaises(Exception, State, pos=np.array([31]))
        self.assertRaises(Exception, State, vel=np.array([31]))
        self.assertRaises(Exception, State, acc=np.array([31, -30, 50.0, 12]))
        self.assertRaises(Exception, State, ang_p=np.array([3, 1, 13, 15, 98]))
        self.assertRaises(Exception, State, ang_v=np.array([3, 2, 5, 53]))
        self.assertRaises(Exception, State, ang_a=np.array([3, 52, True, 3]))

if __name__ == '__main__':
    unittest.main()
