
from robot_brain.state import State

class HLogger:
    """ Logger class which collects and outputs/saves data of the hypothesis graph. """ 




    def __init__(self):

        print('logger initialised')
        self.data = {}


    def setup(self, task):
        """ create dictionary with placeholder for every subtask in the task. """
        for (subtask_nmr, (obst, target_state)) in enumerate(task):
            self.data["subtask_"+str(subtask_nmr)] = {
                    "obstacle_name": obst.name,
                    "target_state_pos_2d": target_state.get_2d_pose(),
                    "hypothesis": {},
                    }
        print(self.data)


    def add_edge_pred_err(self, pred_err):
        print('adding prediction error '+pred_err)


    def print_logs(self):
        print('printing the logs')

    def save_logs(self):
        print('saving the logs')
