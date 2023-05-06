import os
import numpy as np
import json
from pygments import highlight, lexers, formatters
from pathlib import Path

from robot_brain.global_planning.edge import Edge
from robot_brain.global_variables import PROJECT_PATH, SAVE_LOG_METRICS

class HLogger:
    """ Logger class which collects and outputs/saves data of the hypothesis graph. """

    def __init__(self):
        self.data = {
            "completed": False,
            "subtasks": {},
            "total_time": None,
            "execute_time": None,
            "search_time": None}


    def setup(self, task: dict):
        """ create dictionary with placeholder for every subtask in the task. """
        assert isinstance(task, dict), f"task should be a dictionary and is a {type(task)}"

        for (subtask_name, (obst, target_state)) in task.items():
            self.data["subtasks"][subtask_name] = {
                    "obstacle_name": obst.name,
                    "target_state_pos_2d": target_state.get_2d_pose().tolist(),
                    "completed": False,
                    "num_hypotheses": 0,
                    "hypotheses": {}}

    def add_succesfull_hypothesis(self, hypothesis: list, subtask: dict):
        """ add a finished succesfyll hypothesis to the logger. """

        assert isinstance(hypothesis, list), f"hypothesis should be a list and is a {type(hypothesis)}"
        assert all(isinstance(edge, Edge) for edge in hypothesis), "hypothesis should only contain Edges"
        assert isinstance(subtask, dict), f"subtask should be a typle and is {type(subtask)}"

        if subtask["target_node"].subtask_name is None:
            raise ValueError("subtask_name was not set.")
        else:
            subtask_name = subtask["target_node"].subtask_name

        self.data["subtasks"][subtask_name]["completed"] = True
        self.data["subtasks"][subtask_name]["num_hypotheses"] += 1

        # TODO: final position of the drive/push
        hypothesis_log = {
                "edges": {},
                "num_edges":  len(hypothesis),
                "completed" : True,
                "search_time" : subtask["search_time"],
                "execute_time" : subtask["execute_time"],
                "total_time" : subtask["search_time"] + subtask["execute_time"],
                }

        for (edge_nmr, edge) in enumerate(hypothesis):
            hypothesis_log["edges"]["edge_"+str(edge_nmr)] = edge.create_log()

        hypothesis_key = "hypothesis_"+str(len(self.data["subtasks"][subtask_name]["hypotheses"])+1)

        self.data["subtasks"][subtask_name]["hypotheses"][hypothesis_key] = hypothesis_log

    def add_failed_hypothesis(self, hypothesis, subtask, fail_reason):
        """ add hypothesis which failed to logs. """

        assert isinstance(hypothesis, list), f"hypothesis should be a list and is a {type(hypothesis)}"
        assert all(isinstance(edge, Edge) for edge in hypothesis), "hypothesis should only contain Edges"
        assert isinstance(subtask, dict), f"subtask should be a typle and is {type(subtask)}"

        if subtask["target_node"].subtask_name is None:
            raise ValueError("subtask_name was not set.")
        else:
            subtask_name = subtask["target_node"].subtask_name

        self.data["subtasks"][subtask_name]["completed"] = False
        self.data["subtasks"][subtask_name]["num_hypotheses"] += 1

        # TODO: final position of the drive/push
        hypothesis_log = {
                "edges": {},
                "num_edges":  len(hypothesis),
                "completed" : False,
                "fail_reason": fail_reason,
                "search_time" : subtask["search_time"],
                "execute_time" : subtask["execute_time"],
                "total_time" : subtask["search_time"] + subtask["execute_time"],
                }

        for (edge_nmr, edge) in enumerate(hypothesis):
            hypothesis_log["edges"]["edge_"+str(edge_nmr)] = edge.create_log()

        hypothesis_key = "hypothesis_"+str(len(self.data["subtasks"][subtask_name]["hypotheses"])+1)

        self.data["subtasks"][subtask_name]["hypotheses"][hypothesis_key] = hypothesis_log

    def complete_log_succes(self, success_ratio: float):
        """ finish up log after succesfully finishing task. """

        self.data["hypothesis_success_ratio"] = success_ratio
        self.data["completed"] = True
        self.compute_total_time()

    def complete_log_failed(self):
        """ finish up log after a failing to complete a task. """
        self.compute_total_time()

    def compute_total_time(self):
        """ sum up the time spend in the execution/searching loop for every hypothesis. """

        task_search_time = 0
        task_execute_time = 0

        for subtask in self.data["subtasks"].values():
            subtask_search_time = 0
            subtask_execute_time = 0

            for hypothesis in subtask["hypotheses"].values():

                temp_stime = hypothesis["search_time"]
                temp_etime = hypothesis["execute_time"]
                assert isinstance(temp_stime, float), f"search_time should be a float and is a {type(temp_stime)}"
                assert isinstance(temp_etime, float), f"execute_time should be a float and is a {type(temp_etime)}"
                subtask_search_time += temp_stime
                subtask_execute_time += temp_etime

            task_search_time += subtask_search_time
            task_execute_time += subtask_execute_time
            subtask["search_time"] = subtask_search_time
            subtask["execute_time"] = subtask_execute_time
            subtask["total_time"] = subtask_search_time + subtask_execute_time

        self.data["search_time"] = task_search_time
        self.data["execute_time"] = task_execute_time
        self.data["total_time"] = task_search_time + task_execute_time

    def print_logs(self):
        """ prints the logs in JSON format. """
        formatted_json = json.dumps(self.data, sort_keys=False, indent=2)
        colorful_json = highlight(formatted_json, lexers.JsonLexer(), formatters.TerminalFormatter())
        print(colorful_json)

    def save_logs(self):
        """ converts the logs to JSON and saves it under a unique new name. """

        if SAVE_LOG_METRICS:
            dir_path = PROJECT_PATH+"logger/logs"
            unique_id_num = len([entry for entry in os.listdir(dir_path) if os.path.isfile(os.path.join(dir_path, entry))])+1

            unique_dir_path = dir_path+"/task_log_"+str(unique_id_num)+".json"

            # if file exist, add _v2 to the name
            if Path(unique_dir_path).is_file():
                unique_dir_path = unique_dir_path[0:-5] + "_v2.json"

            with open(unique_dir_path, "x") as outfile:
                json.dump(self.data, outfile)

