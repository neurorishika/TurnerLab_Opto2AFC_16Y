import numpy as np
import json
import time
from sixteeny.utils.printer import Printer

class ArenaTracker(object):
    def __init__(self, arena_index, experimenter, controllers, printer=None):
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
        self.experiment_folder = self.experimenter.experiment_folder

        self.trial_count = -1
        self.frame_count = -1

        self.completed = False
        self.started = False

        self.start_arm = None

        self.fly_position = np.array([0, 0])
        self.current_arm = None  # absolute arm coordinates (0-2)

        self.in_reward_zone = False
        self.time_enter_reward_zone = None

        # initialize tracker matrices
        self.n_trials = experimenter.n_trials
        self.max_frames = experimenter.n_trials * 50 * 180

        self.fly_positions = np.zeros((self.max_frames, 2)) * np.nan
        self.frame_times = np.zeros(self.max_frames) * np.nan
        self.current_arms = np.zeros(self.max_frames) * np.nan
        self.current_trials = np.zeros(self.max_frames) * np.nan
        self.current_reward_zone_status = np.zeros(self.max_frames) * np.nan

        self.trial_start_times = np.zeros(self.n_trials) * np.nan
        self.trial_end_times = np.zeros(self.n_trials) * np.nan

        self.trial_odor_start_delay = np.zeros(self.n_trials) * np.nan

        self.time_spent_in_reward_zone = np.zeros(self.n_trials)
        self.odor_vectors = np.zeros((self.n_trials, 3)) * np.nan
        self.start_arms = np.zeros(self.n_trials)

        self.trial_baited = np.zeros(self.n_trials)
        self.reward_states = np.zeros((self.n_trials, 3))

        self.chosen_arms = np.zeros(self.n_trials)
        self.chosen_odor = np.zeros(self.n_trials)
        self.reward_delivered = np.zeros(self.n_trials)

        self.lengths_of_trials = np.zeros(self.n_trials)

        self.clockwise_arena_indices = [0, 2, 4, 6, 8, 10, 12, 14]

        self.start_new_trial_next_frame = False

        if printer is not None:
            if not isinstance(printer, Printer):
                raise TypeError("printer must be an instance of the Printer class")
        
        self.printer = printer

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

        if self.start_new_trial_next_frame:
            self.start_new_trial_next_frame = False
            self.start_new_trial()

        # increment frame count
        self.frame_count += 1

        # update tracker matrices
        self.fly_positions[self.frame_count, :] = current_position
        self.frame_times[self.frame_count] = time.time()
        self.current_arms[self.frame_count] = current_arm
        self.current_trials[self.frame_count] = self.trial_count
        self.current_reward_zone_status[self.frame_count] = in_reward_zone

        self.fly_position = current_position
        self.current_arm = current_arm

        rewarded = False

        if not self.completed and self.started:
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

            if time_needed_in_reward_zone == "inf":
                time_needed_in_reward_zone = np.inf
            else:
                time_needed_in_reward_zone = float(time_needed_in_reward_zone)

            if self.timed == 0.0:
                # check if in reward zone for the required time
                if in_reward_zone:
                    if time_needed_in_reward_zone == 0 or (
                        time_needed_in_reward_zone > 0
                        and (time.time() - self.time_enter_reward_zone) > time_needed_in_reward_zone
                    ):
                        rewarded = True
                        self.administer_reward()
                        # self.start_new_trial()
                        self.start_new_trial_next_frame = True
            else:

                # process odor delays
                if self.odor_delay > 0.0 and time.time() - self.trial_start_time > self.odor_delay:
                    self.controllers["odor"].publish(self.arena_index, self.odor_vector)
                    self.trial_odor_start_delay[self.trial_count] = time.time() - self.trial_start_time

                # process timed reward
                if time.time() - self.trial_start_time > self.timed:
                    rewarded = True
                    self.administer_reward()
                    # self.start_new_trial()
                    self.start_new_trial_next_frame = True

        if not self.started:
            self.start_new_trial()
            self.started = True

        return rewarded

    def administer_reward(self):
        # update tracker matrices
        if self.timed == 0.0:
            self.time_spent_in_reward_zone[self.trial_count] = time.time() - self.time_enter_reward_zone
        else:
            self.time_spent_in_reward_zone[self.trial_count] = (
                time.time() - self.trial_start_time
            )  # the entire area is the reward zone

        self.chosen_arms[self.trial_count] = self.absolute_to_relative_arm(
            self.current_arm, self.start_arm, self.arena_index
        )
        self.chosen_odor[self.trial_count] = self.odor_vector[self.current_arm]

        # sample reward state
        if self.timed == 0.0:
            reward_state = [np.random.choice(2, p=[1 - i, i]) for i in self.reward_probability]

            # check if trial is baited, if so, get the logical OR of past reward states
            if self.trial_baited[self.trial_count] == 1:
                reward_state = (
                    np.int32(np.logical_or(reward_state, self.reward_states[self.trial_count - 1, :]))
                    if self.trial_count > 0
                    else reward_state
                )
        else:
            reward_state = [1 for i in range(len(self.reward_probability))]

        if self.timed == 0.0:
            reward_stimulus = (
                self.experiment_folder + "stimuli/" + self.reward_stimulus[self.odor_vector[self.current_arm]]
            )
        else:
            reward_stimulus = self.experiment_folder + "stimuli/" + self.unconditioned_stimulus

        # sample reward
        if reward_state[self.odor_vector[self.current_arm]] == 1:
            # load json file
            with open(reward_stimulus, "r") as f:
                reward_stimulus = json.load(f)
            # deliver reward
            self.controllers["led"].accumulate_json(self.arena_index, reward_stimulus, debug_mode=False)
            self.reward_delivered[self.trial_count] = 1
            # reset reward state
            reward_state[self.odor_vector[self.current_arm]] = 0

        # update reward states
        self.reward_states[self.trial_count, :] = reward_state

    def start_new_trial(self):

        if self.trial_count >= 0:
            self.trial_end_times[self.trial_count] = time.time()
            self.lengths_of_trials[self.trial_count] = self.trial_end_times[self.trial_count] - self.trial_start_time

        # check if all trials have been completed
        if self.trial_count == self.n_trials - 1:
            if self.printer is not None:
                self.printer.print("All trials completed")
            else:
                print("All trials completed")
            self.trial_count += 1
            # flip all valves to air
            self.controllers["odor"].publish(self.arena_index, [0, 0, 0])
            self.completed = True
            return

        # increment trial count
        self.trial_count += 1

        # reset trial timer
        self.trial_start_time = time.time()
        self.trial_start_times[self.trial_count] = self.trial_start_time

        # update air arm to current arm
        self.start_arm = self.current_arm
        self.start_arms[self.trial_count] = self.start_arm

        # reset reward zone timer
        self.in_reward_zone = False
        self.time_enter_reward_zone = None

        # get next trial data
        history = {
            "trial_count": self.trial_count,
            "chosen_arms": self.chosen_arms[: self.trial_count] if self.trial_count > 0 else [],
            "chosen_odor": self.chosen_odor[: self.trial_count] if self.trial_count > 0 else [],
            "reward_delivered": self.reward_delivered[: self.trial_count + 1],
            "time_spent_in_reward_zone": self.time_spent_in_reward_zone[: self.trial_count + 1],
            "lengths_of_trials": self.lengths_of_trials[: self.trial_count + 1],
        }
        next_trial = self.experimenter.get_next_trial(history)

        # update data for next trial
        new_indices = [self.absolute_to_relative_arm(i, self.start_arm, self.arena_index) for i in range(3)]

        self.odor_vector = [
            int(next_trial["relative_odor_vector"][i]) for i in new_indices
        ]  # odor vectors for each absolute arm

        self.odor_vectors[self.trial_count, :] = self.odor_vector

        self.time_needed_in_reward_zone = next_trial["time_needed_in_reward_zone"]  # indexed wrt odors
        self.reward_stimulus = next_trial["reward_stimulus"]  # indexed wrt odors
        self.reward_probability = next_trial["reward_probability"]  # indexed wrt odors

        self.trial_baited[self.trial_count] = next_trial["baited"]  # 0 or 1 depending on if trial was baited

        self.timed = float(next_trial["timed"])  # 0 or positive number in seconds if timed
        self.odor_delay = float(next_trial["odor_delay"])  # seconds
        self.unconditioned_stimulus = next_trial["unconditioned_stimulus"]  # json file

        # flip the valves to the new odor vector
        if self.odor_delay == 0.0:
            self.controllers["odor"].publish(self.arena_index, self.odor_vector)
            self.trial_odor_start_delay[self.trial_count] = time.time() - self.trial_start_time
        else:
            self.controllers["odor"].publish(self.arena_index, [0, 0, 0])

    def save_data(self, directory):
        """
        Saves data to a .ydata file.
        """
        # remove the last few NaN values from the data
        self.fly_positions = self.fly_positions[: self.frame_count + 1, :]
        self.frame_times = self.frame_times[: self.frame_count + 1]
        self.current_arms = self.current_arms[: self.frame_count + 1]
        self.current_trials = self.current_trials[: self.frame_count + 1]
        self.current_reward_zone_status = self.current_reward_zone_status[: self.frame_count + 1]
        # remove the data for the unused trials
        self.chosen_arms = self.chosen_arms[: self.trial_count]
        self.chosen_odor = self.chosen_odor[: self.trial_count]
        self.reward_delivered = self.reward_delivered[: self.trial_count]
        self.time_spent_in_reward_zone = self.time_spent_in_reward_zone[: self.trial_count]
        self.lengths_of_trials = self.lengths_of_trials[: self.trial_count]
        # save data as .ydata file in json format
        data = {
            "fly_positions": self.fly_positions.tolist(),
            "frame_times": self.frame_times.tolist(),
            "current_arms": self.current_arms.tolist(),
            "current_trial": self.current_trials.tolist(),
            "current_reward_zone_status": self.current_reward_zone_status.tolist(),
            "chosen_arms": self.chosen_arms.tolist(),
            "chosen_odor": self.chosen_odor.tolist(),
            "reward_delivered": self.reward_delivered.tolist(),
            "trial_start_times": self.trial_start_times.tolist(),
            "trial_end_times": self.trial_end_times.tolist(),
            "trial_odor_start_delay": self.trial_odor_start_delay.tolist(),
            "time_spent_in_reward_zone": self.time_spent_in_reward_zone.tolist(),
            "lengths_of_trials": self.lengths_of_trials.tolist(),
            "odor_vectors": self.odor_vectors.tolist(),
            "trial_baited": self.trial_baited.tolist(),
            "reward_states": self.reward_states.tolist(),
            "start_arms": self.start_arms.tolist(),
            "n_trials": self.n_trials,
            "max_frame_count": self.frame_count,
            "trial_count": self.trial_count,
            "experiment_states": self.experimenter.get_all_states(),
        }
        with open(directory + "fly_{}.ydata".format(self.arena_index), "w") as f:
            json.dump(data, f)
