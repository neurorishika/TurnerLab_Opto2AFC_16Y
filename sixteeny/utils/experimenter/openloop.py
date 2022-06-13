from sixteeny.utils.experimenter.base import Experimenter
import pandas as pd


class CSVExperimenter(Experimenter):
    """
    A class that used an CSV config file to design experiments
    """

    def __init__(self, experiment_config_file):
        """
        A method to initialize an CSVExperimenter object
        
        Variables:
            experiment_config_file: full path to csv file with details of experiment
        """
        super().__init__(experiment_config_file)
        # confirm that the experiment config file is a csv file
        assert self.experiment_config_file.endswith(".csv")
        # read the experiment config file
        self.experiment_config = pd.read_csv(self.experiment_config_file)
        # get experiment folder as the full path to the folder containing the experiment config file
        self.experiment_folder = self.experiment_config_file.split("/")[:-2]
        self.experiment_folder = "/".join(self.experiment_folder) + "/"
        self.n_trials = self.experiment_config.shape[0]

    def get_next_trial(self, history):
        """
        Return the next trial from the csv
        """
        # get the next trial from the csv
        next_trial = {}
        next_trial["reward_probability"] = list(
            self.experiment_config.iloc[self.trial_number][["P(R|Air)", "P(R|O1)", "P(R|O2)"]]
        )
        next_trial["relative_odor_vector"] = list(
            self.experiment_config.iloc[self.trial_number][["Odor(Start)", "Odor(Left)", "Odor(Right)"]]
        )
        next_trial["time_needed_in_reward_zone"] = list(
            self.experiment_config.iloc[self.trial_number][["StayTime(Air)", "StayTime(O1)", "StayTime(O2)"]]
        )
        next_trial["reward_stimulus"] = list(
            self.experiment_config.iloc[self.trial_number][["CStim(Air)", "CStim(O1)", "CStim(O2)"]]
        )
        next_trial["timed"] = self.experiment_config.iloc[self.trial_number]["Timed"]
        next_trial["odor_delay"] = self.experiment_config.iloc[self.trial_number]["OdorDelay"]
        next_trial["unconditioned_stimulus"] = self.experiment_config.iloc[self.trial_number]["UStim"]
        # increment the trial number
        self.trial_number += 1
        # add to states
        self.states.append(next_trial)
        return next_trial

