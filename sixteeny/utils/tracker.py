import numpy as np
import json
import time


class ArenaTracker(object):
    def __init__(self, arena_index, experimenter, controllers):
        """
        A class to track the state of each of the 16 arenas.

        Variables:
            arena_index: the index of the arena to track (int)
            experimenter: an instance of the Experimenter class to get experiment definitions
            controllers: a dictionary of controllers to send commands to the arena (dict)
        """
        self.arena_index = arena_index
        self.experimenter = experimenter
        self.controllers = controllers

        self.trial_count = -1
        self.frame_count = -1

        self.completed = False

        self.start_arm = None

        self.fly_position = np.array([0, 0])
        self.current_arm = None  # absolute arm coordinates (0-2)

        self.in_reward_zone = False
        self.time_enter_reward_zone = None

        # initialize tracker matrices
        self.n_trials = 100
        self.max_frames = 100 * 60 * 120
        self.fly_positions = np.zeros((self.max_frames, 2)) * np.nan
        self.frame_times = np.zeros(self.max_frames) * np.nan

        self.time_spent_in_reward_zone = np.zeros(self.n_trials)

        self.chosen_arms = np.zeros(self.n_trials)
        self.chosen_odor = np.zeros(self.n_trials)
        self.reward_delivered = np.zeros(self.n_trials)

        self.lengths_of_trials = np.zeros(self.n_trials)

        self.clockwise_arena_indices = [1, 3, 5, 7, 9, 11, 13, 15]

    def relative_to_absolute_arm(self, relative_position, start_arm, arena_index):
        """
        Converts a relative position to an absolute position.

        Variables:
            relative_position: the position to convert (0-Start, 1-Left, 2-Right)
            start_arm: the absolute arm where the trial started (0-2)
            arena_index: the index of the arena to convert to absolute position
        """
        if arena_index in self.clockwise_arena_indices:
            if relative_position == 0:
                return start_arm
            elif relative_position == 1:
                return (start_arm + 1) % 3
            elif relative_position == 2:
                return (start_arm - 1) % 3
        else:
            if relative_position == 0:
                return start_arm
            elif relative_position == 1:
                return (start_arm - 1) % 3
            elif relative_position == 2:
                return (start_arm + 1) % 3

    def absolute_to_relative_arm(self, absolute_arm, start_arm, arena_index):
        """
        Converts an absolute arm to a relative arm.

        Variables:
            absolute_arm: the absolute arm to convert (0-2)
            start_arm: the absolute arm where the trial started (0-2)
            arena_index: the index of the arena to convert to relative arm
        """
        if arena_index in self.clockwise_arena_indices:
            if absolute_arm == start_arm:
                return 0
            elif absolute_arm == (start_arm + 1) % 3:
                return 1
            elif absolute_arm == (start_arm - 1) % 3:
                return 2
        else:
            if absolute_arm == start_arm:
                return 0
            elif absolute_arm == (start_arm - 1) % 3:
                return 1
            elif absolute_arm == (start_arm + 1) % 3:
                return 2

    def update(self, current_arm, current_position, in_reward_zone):

        # increment frame count
        self.frame_count += 1

        # update tracker matrices
        self.fly_positions[self.frame_count, :] = current_position
        self.frame_times[self.frame_count] = time.time()

        self.fly_position = current_position
        self.current_arm = current_arm

        if not self.completed:
            # start reward zone timer

            # if entering reward zone
            if not self.in_reward_zone and in_reward_zone:
                self.time_enter_reward_zone = time.time()
            # if leaving reward zone before time condition is met
            elif self.in_reward_zone and not in_reward_zone:
                self.time_enter_reward_zone = None

            # update reward zone status
            self.in_reward_zone = in_reward_zone

            # if reward condition is met, start new trial
            time_needed_in_reward_zone = self.time_needed_in_reward_zone[self.odor_vector[self.current_arm]]

            if (time_needed_in_reward_zone == 0 and self.in_reward_zone) or (
                time_needed_in_reward_zone > 0
                and time.time() - self.time_enter_reward_zone > time_needed_in_reward_zone
            ):
                self.administer_reward()
                self.start_new_trial()

    def administer_reward(self):
        # update tracker matrices
        self.time_spent_in_reward_zone[self.trial_count] = time.time() - self.time_enter_reward_zone
        self.reward_delivered[self.trial_count] = 1
        self.chosen_arms[self.trial_count] = self.absolute_to_relative_arm(
            self.current_arm, self.start_arm, self.arena_index
        )
        self.chosen_odor[self.trial_count] = self.odor_vector[self.current_arm]

        reward_probability = self.reward_probability[self.odor_vector[self.current_arm]]
        reward_stimulus = self.reward_stimulus[self.odor_vector[self.current_arm]]

        # sample reward
        if np.random.rand() < reward_probability:
            # load json file
            with open(self.reward_stimulus, "r") as f:
                reward_stimulus = json.load(f)
            # deliver reward
            self.controllers["led"].accumulate_json(self.arena_index, reward_stimulus)
        pass

    def start_new_trial(self):

        # check if all trials have been completed
        if self.trial_count == self.n_trials - 1:
            print("All trials completed")
            self.completed = True
            return

        if self.trial_count >= 0:
            self.lengths_of_trials[self.trial_count] = time.time() - self.trial_start_time

        # increment trial count
        self.trial_count += 1

        # reset trial timer
        self.trial_start_time = time.time()

        # update air arm to current arm
        self.start_arm = self.current_arm

        # reset reward zone timer
        self.in_reward_zone = False
        self.time_enter_reward_zone = None

        # get next trial data
        history = {
            "trial_count": self.trial_count,
            "chosen_arms": self.chosen_arms[: self.trial_count],
            "chosen_odor": self.chosen_odor[: self.trial_count],
            "reward_delivered": self.reward_delivered[: self.trial_count],
            "time_spent_in_reward_zone": self.time_spent_in_reward_zone[: self.trial_count],
            "lengths_of_trials": self.lengths_of_trials[: self.trial_count],
        }
        next_trial = self.experimenter.get_next_trial(history)

        ### TODO: update data for next trial
        new_indices = [self.relative_to_absolute_arm(i, self.start_arm, self.arena_index) for i in range(3)]
        self.odor_vector = next_trial["relative_odor_vector"][new_indices]  # odor vectors for each absolute arm

        # flip the valves to the new odor vector
        self.controllers["odor"].publish(self.arena_index, self.odor_vector)

        self.time_needed_in_reward_zone = next_trial["time_needed_in_reward_zone"]  # indexed wrt odors
        self.reward_stimulus = next_trial["reward_stimulus"]  # indexed wrt odors
        self.reward_probability = next_trial["reward_probability"]  # indexed wrt odors

    def save_data(self, directory):
        """
        Saves data to a .ydata file.
        """
        # save data as .ydata file in json format
        data = {
            "fly_positions": self.fly_positions,
            "frame_times": self.frame_times,
            "chosen_arms": self.chosen_arms,
            "chosen_odor": self.chosen_odor,
            "reward_delivered": self.reward_delivered,
            "time_spent_in_reward_zone": self.time_spent_in_reward_zone,
            "lengths_of_trials": self.lengths_of_trials,
            "n_trials": self.n_trials,
            "max_frame_count": self.frame_count,
        }
        with open(directory + "fly_{}.ydata".format(self.fly_id), "w") as f:
            json.dump(data, f)
