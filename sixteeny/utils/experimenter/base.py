from csv import list_dialects
import numpy as np


class Experimenter(object):
    """
    A class to define any sequence of trials in an open-loop or closed-loop setup
    """

    def __init__(self, experiment_config_file):
        """
        A method to initialize an experimenter object

        Variables:
            experiment_config_file: path to file with details of experiment
        """
        self.experiment_config_file = experiment_config_file
        self.trial_number = 0
        self.states = []

    def get_next_trial(self, history):
        """
        An example of the method to return the next trial to be run
        """
        self.trial_number += 1
        empty_stim = "empty.stim"
        next_trial = {
            "reward_probability": [0.5, 0.5, 0.5],
            "relative_odor_vector": [0, 1, 2],
            "time_needed_in_reward_zone": ["inf", 0, 0],
            "reward_stimulus": [empty_stim, empty_stim, empty_stim],
            "timed": False,
            "odor_delay": 0,
            "unconditioned_stimulus": empty_stim,
        }
        self.states.append(next_trial)
        return next_trial

    def get_all_states(self):
        """
        Return the full state history of the experiment
        """
        list_of_states = []
        for state in self.states:
            state_dict = {}
            for key, value in state.items():
                if isinstance(value, list):
                    values = []
                    for item in value:
                        values.append(str(item))
                    state_dict[key] = values
                else:
                    state_dict[key] = str(value)
            list_of_states.append(state_dict)
        # list_of_states = [list(map(list, state.items())) for state in self.states]
        return list_of_states
