from selectors import SelectorKey
from sixteeny.utils.experimenter.base import Experimenter
import json
import numpy as np


class FiniteStateExperimenter(Experimenter):
    """
    A class that uses a Markov chain on State-Action space to design experiments
    """

    def __init__(self, experiment_config_file):
        """
        A method to initialize a FiniteStateExperimenter object
        """
        super().__init__(experiment_config_file)
        # confirm that the config file is a json object
        assert self.experiment_config_file.endswith(".yfse")
        # read the config file
        with open(self.experiment_config_file) as f:
            self.experiment_config = json.load(f)
        # get experiment folder as the full path to the folder containing the experiment config file
        self.experiment_folder = self.experiment_config_file.split("/")[:-2]
        self.experiment_folder = "/".join(self.experiment_folder) + "/"
        # get experiment details
        self.n_trials = int(self.experiment_config["num_trials"])
        self.n_states = int(self.experiment_config["num_states"])
        self.state_labels = np.array(self.experiment_config["state_labels"])
        self.reward_probabilities = np.array(self.experiment_config["reward_probabilities"])  # shape (2,2)
        self.odor_transition_matrix = np.array(
            self.experiment_config["odor_transition_matrix"]
        )  # shape (2, n_states, n_states)
        self.rewarded_stimulus = self.experiment_config["rewarded_stimulus"]
        self.unrewarded_stimulus = self.experiment_config["unrewarded_stimulus"]
        # randomly choose a starting state
        self.current_state = np.random.choice(self.n_states)
        self.current_state_label = self.state_labels[self.current_state]
        self.current_reward_probability = self.reward_probabilities[self.current_state_label]

    def state_update(self, chosen_odor):
        """
        Update the current state based on the chosen odor
        """
        # get the next state
        self.current_state = np.random.choice(
            self.n_states, p=self.odor_transition_matrix[int(chosen_odor - 1), self.current_state]
        )
        self.current_state_label = self.state_labels[self.current_state]
        self.current_reward_probability = self.reward_probabilities[self.current_state_label]

    def get_next_trial(self, history):
        """
        Return the next trial based on current state
        """
        # get latest choice
        chosen_odor = history["chosen_odor"][-1]
        # update the current state
        self.state_update(chosen_odor)
        # get the next trial
        next_trial = {}
        next_trial["reward_probability"] = [0, self.current_reward_probability[0], self.current_reward_probability[1]]
        randomize = np.random.choice(2)
        next_trial["relative_odor_vector"] = [0, 1 + randomize, 2 - randomize]
        next_trial["time_needed_in_reward_zone"] = ["inf", "0", "0"]
        next_trial["reward_stimulus"] = ["empty.stim", self.rewarded_stimulus, self.unrewarded_stimulus]
        next_trial["timed"] = 0
        next_trial["odor_delay"] = 0
        next_trial["unconditioned_stimulus"] = "empty.stim"
        return next_trial
