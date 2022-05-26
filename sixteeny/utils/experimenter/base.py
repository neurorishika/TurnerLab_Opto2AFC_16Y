import numpy as np

class Experimenter(object):
    """
    A class to define any sequence of trials in an open-loop or closed-loop setup
    """
    def __init__(self,experiment_config_file):
        """
        A method to initialize an experimenter object

        Variables:
            experiment_config_file: path to file with details of experiment
        """
        self.experiment_config_file = experiment_config_file
        self.trial_number = 0
        self.states = []
    
    def get_next_trial(self,history):
        """
        An example of the method to return the next trial to be run
        """
        self.trial_number += 1
        empty_stim = {
            'color': '#000000',
            'intensity': 0,
            'pulse_width': 1,
            'pulse_period': 1,
            'pulse_count': 1,
            'pulse_delay': 0,
            'pulse_deadtime': 0,
            'pulse_repeat': 1,
        }
        next_trial = {
            'reward_probability': [0.5,0.5,0.5],
            'relative_odor_vector': [0,1,2],
            'time_needed_in_reward_zone': [np.inf,0,0],
            'reward_stimulus': [empty_stim,empty_stim,empty_stim],
            'timed': False,
            'odor_delay': 0,
            'unconditioned_stimulus': empty_stim,
        }
        self.state.append(next_trial)
        return next_trial