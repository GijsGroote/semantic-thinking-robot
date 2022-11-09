import numpy as np
from robot_brain.state import State

class HLogger:
    """ Logger class which collects and outputs/saves data of the hypothesis graph. """ 




    def __init__(self):

        print('logger initialised')
        self.data = {}


    def setup(self, task):
        """ create dictionary with placeholder for every subtask in the task. """
        for (subtask_name, (obst, target_state)) in task.items():
            self.data[subtask_name] = {
                    "obstacle_name": obst.name,
                    "target_state_pos_2d": target_state.get_2d_pose(),
                    "hypothesis": {},
                    }
        print(self.data)

    def update_succesfull_hypothesis(self, hypothesis, subtask):
        """ update the logger with a finished successfull hypothesis. """
        print("some means which would ")

        print(f"should be starting subtas naem {subtask[0].name}")
        print(subtask[0].subtask_name)
        print(f"should be target subtas naem {subtask[1].name}")
        print()

        if subtask[1].subtask_name is None:
            raise ValueError("please set subtask_name, make sure it is always set")
        else:
            subtask_name = subtask[1].subtask_name
            self.data[subtask_name]["edges"] = {}

        for (edge_nmr, edge) in enumerate(hypothesis):
            # self.da
            self.data[subtask_name]["edges"]["edge_"+str(edge_nmr)] = {"average_pred_error": np.average(edge.controller.pred_error),
                        "maximal_pred_error": max(edge.controller.pred_error),
                        "minimum_pred_error": min(edge.controller.pred_error[1:-1])}

    def add_edge_pred_err(self, pred_err):
        print('adding prediction error '+pred_err)


    def print_logs(self):
        print(f"The task has {len(self.data)} subtasks:")
        for (subtask_name, subtask) in self.data.items():
            print(f"{subtask_name} with {len(subtask['edges'])} edges:")
            for (edge_name, edge) in subtask["edges"].items():
                print(f"{edge_name}, has max(pred_error) = {edge['maximal_pred_error']}, min(pred_error) = {edge['minimum_pred_error']}, average(pred_error) = {edge['average_pred_error']}")
            print()

    def save_logs(self):
        print('saving the logs')
