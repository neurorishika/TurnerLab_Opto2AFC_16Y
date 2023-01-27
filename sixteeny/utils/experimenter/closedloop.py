from sixteeny.utils.experimenter.base import Experimenter
import json
import numpy as np


class DeterministicFiniteStateExperimenter(Experimenter):
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
        self.exp_trials = int(self.experiment_config["num_trials"])
        self.naive_trials = int(self.experiment_config["num_naive_trials"])
        self.n_trials = self.exp_trials + self.naive_trials
        self.n_states = int(self.experiment_config["num_states"])
        self.state_labels = np.array(self.experiment_config["state_labels"]) # 0 is rewarded, 1 is unrewarded
        self.transition_matrix = np.array(
            self.experiment_config["state_transitions"]
        )  # shape (n_states, 2)
        self.rewarded_stimulus = self.experiment_config["rewarded_stimulus"]
        self.unrewarded_stimulus = self.experiment_config["unrewarded_stimulus"]
        self.lr_randomization = self.experiment_config["lr_randomization"]
        # randomly choose a starting state
        self.starting_state = self.experiment_config["starting_state"]
        if self.starting_state == "-1":
            self.current_state = np.random.choice(self.n_states)
        else:
            # ensure that the starting state is valid
            assert int(self.starting_state) < self.n_states
            self.current_state = int(self.starting_state)

    def state_update(self, chosen_odor):
        """
        Update the current state based on the chosen odor
        """
        # get the next state
        self.current_state = self.transition_matrix[self.current_state, int(chosen_odor - 1)]

    def get_next_trial(self, history, debug = False):
        """
        Return the next trial based on current state
        """        
        # get latest choice if there is one
        if debug:
            print("State before update: ", self.current_state)
        if len(history["chosen_odor"]) < self.naive_trials:
            self.rewards = [0, 0]
            state = -1
        elif len(history["chosen_odor"]) == self.naive_trials:
            self.rewards = [self.state_labels[self.transition_matrix[self.current_state][0]], self.state_labels[self.transition_matrix[self.current_state][1]]]
            state = self.current_state
        else:
            chosen_odor = int(history["chosen_odor"][-1])
            if debug:
                print("Chosen odor: ", chosen_odor, "History: ", history["chosen_odor"], "trial number: ", history["trial_count"])
            # update the current state
            self.state_update(chosen_odor)
            self.rewards = [self.state_labels[self.transition_matrix[self.current_state][0]], self.state_labels[self.transition_matrix[self.current_state][1]]]
            state = self.current_state
        if debug:
            print("State after update: ", state, "Rewards: ", self.rewards)

        # get the next trial
        next_trial = {}
        randomize = np.random.choice(2) if self.lr_randomization else 0
        next_trial["world_state"] = state
        next_trial["world_rewards"] = self.rewards
        next_trial["relative_odor_vector"] = [0, 1 + randomize, 2 - randomize]
        next_trial["reward_probability"] = [
            0,
            self.rewards[randomize],
            self.rewards[1 - randomize],
        ]  # adjust from randomization
        next_trial["time_needed_in_reward_zone"] = ["inf", "0", "0"]
        next_trial["reward_stimulus"] = [
            "empty.stim", 
            self.rewarded_stimulus if self.rewards[randomize] == 1 else self.unrewarded_stimulus, 
            self.rewarded_stimulus if self.rewards[1 - randomize] == 1 else self.unrewarded_stimulus
            ]
        next_trial["timed"] = 0
        next_trial["odor_delay"] = 0
        next_trial["unconditioned_stimulus"] = "empty.stim"
        next_trial["baited"] = 0
        # increment the trial number
        self.trial_number += 1
        # append the trial to the list of trials
        self.states.append(next_trial)
        return next_trial


class ProbabilisticFiniteStateExperimenter(Experimenter):
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
        # get latest choice if there is one
        if len(history["chosen_odor"]) > 0:
            chosen_odor = history["chosen_odor"][-1]
            # update the current state
            self.state_update(chosen_odor)

        # get the next trial
        next_trial = {}
        randomize = np.random.choice(2)
        next_trial["world_state"] = self.current_state
        next_trial["world_state_label"] = self.current_state_label
        next_trial["relative_odor_vector"] = [0, 1 + randomize, 2 - randomize]
        next_trial["reward_probability"] = [
            0,
            self.current_reward_probability[randomize],
            self.current_reward_probability[1 - randomize],
        ]  # adjust from randomization
        next_trial["time_needed_in_reward_zone"] = ["inf", "0", "0"]
        next_trial["reward_stimulus"] = ["empty.stim", self.rewarded_stimulus, self.unrewarded_stimulus]
        next_trial["timed"] = 0
        next_trial["odor_delay"] = 0
        next_trial["unconditioned_stimulus"] = "empty.stim"
        next_trial["baited"] = 0
        self.states.append(next_trial)
        return next_trial