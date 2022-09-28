# a script to clean up the ymaze data

from msilib.schema import Error
from PyQt5 import QtWidgets, QtCore, QtGui
import sys
import os

import numpy as np
import json
from skimage.transform import estimate_transform, AffineTransform
from joblib import Parallel, delayed
import multiprocessing


def euclidean_distance(point1, point2):
    """
    Calculate the euclidean distance between two points
    """
    return np.linalg.norm(point1 - point2)


def realign_lr(start_arm, point, arena_index):
    """
    Aligns the point into a Left-Start-Right coordinate system given the start arm and the arena index (to account for the different CW/CCW alignments)
    """
    clockwise_arenas = [0, 2, 4, 6, 8, 10, 12, 14]
    if start_arm == 1.0:
        transform = AffineTransform(scale=(1.0, 1.0), rotation=0, translation=(0.0, 0.0))
    elif start_arm == 2.0:
        transform = AffineTransform(scale=(1.0, 1.0), rotation=-2 * np.pi / 3, translation=(0.0, 0.0))
    elif start_arm == 0.0:
        transform = AffineTransform(scale=(1.0, 1.0), rotation=2 * np.pi / 3, translation=(0.0, 0.0))
    else:
        raise ValueError("start_arm must be 0, 1 or 2")
    if arena_index in clockwise_arenas:
        return transform(point) * np.array([-1, 1])
    else:
        return transform(point)


def realign_odor(odor_vector, point):
    """
    Aligns the point into an Odor 1-Start-Odor 2 coordinate system given the odor vector
    """
    if np.all(odor_vector == [1.0, 0.0, 2.0]):
        transform = AffineTransform(scale=(1.0, 1.0), rotation=0, translation=(0.0, 0.0))
        return transform(point)
    elif np.all(odor_vector == [2.0, 0.0, 1.0]):
        transform = AffineTransform(scale=(1.0, 1.0), rotation=0, translation=(0.0, 0.0))
        return transform(point) * np.array([-1.0, 1.0])
    elif np.all(odor_vector == [0.0, 1.0, 2.0]):
        transform = AffineTransform(scale=(1.0, 1.0), rotation=2 * np.pi / 3, translation=(0.0, 0.0))
        return transform(point) * np.array([-1.0, 1.0])
    elif np.all(odor_vector == [0.0, 2.0, 1.0]):
        transform = AffineTransform(scale=(1.0, 1.0), rotation=2 * np.pi / 3, translation=(0.0, 0.0))
        return transform(point)
    elif np.all(odor_vector == [1.0, 2.0, 0.0]):
        transform = AffineTransform(scale=(1.0, 1.0), rotation=-2 * np.pi / 3, translation=(0.0, 0.0))
        return transform(point) * np.array([-1.0, 1.0])
    elif np.all(odor_vector == [2.0, 1.0, 0.0]):
        transform = AffineTransform(scale=(1.0, 1.0), rotation=-2 * np.pi / 3, translation=(0.0, 0.0))
        return transform(point)
    elif np.all(odor_vector == [0.0, 0.0, 0.0]):
        return np.array([np.ones_like(point) * np.nan])
    else:
        raise ValueError(
            "odor vector must be [1.0, 0.0, 2.0], [2.0, 0.0, 1.0], [0.0, 1.0, 2.0], [0.0, 2.0, 1.0], [1.0, 2.0, 0.0] or [2.0, 1.0, 0.0]"
        )


def relative_to_absolute_arm(relative_position, start_arm, arena_index):
    if arena_index in [0, 2, 4, 6, 8, 10, 12, 14]:
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


def process_file(file_path, output_path, estimated_transforms, origin):
    """
    Given a ydata file generate processed variables and save them to a new file
    """
    try:
        # get the data as a json file
        with open(file_path, "r") as f:
            data = json.load(f)

        # get index of the arena from the filename
        index = int(file_path.split(".")[0].split("_")[-1])

        # get fly position and switch x and y (because of the transpose)
        fly_position = np.array(data["fly_positions"])
        fly_position = fly_position[:, [1, 0]]

        # get the estimated transform
        transform = estimated_transforms[index]

        # calculate transformed fly position
        print("Calculating transformed fly position for Fly {}".format(index))
        reference_fly_positions = transform(fly_position)

        # interpolate the missing data
        def interpolate(y):
            nans, x = np.isnan(y), lambda z: z.nonzero()[0]
            y[nans] = np.interp(x(nans), x(~nans), y[~nans])
            return y

        print("Interpolating missing data for Fly {}".format(index))
        reference_fly_positions[:, 0] = interpolate(reference_fly_positions[:, 0])
        reference_fly_positions[:, 1] = interpolate(reference_fly_positions[:, 1])

        # calculate instantaneous speed (norm of change in position / delta time)
        print("Calculating instantaneous speed for Fly {}".format(index))
        instantaneous_speed = np.linalg.norm(np.diff(reference_fly_positions, axis=0), axis=1) / np.diff(
            data["frame_times"], axis=0
        )
        instantaneous_speed = np.append(instantaneous_speed, np.nan)

        # calculate instantaneous motion heading (wrt to reference frame)
        print("Calculating instantaneous motion heading for Fly {}".format(index))
        instantaneous_heading = np.arctan2(
            np.diff(reference_fly_positions[:, 1], axis=0), np.diff(reference_fly_positions[:, 0], axis=0)
        )
        instantaneous_heading = np.append(instantaneous_heading, np.nan)

        # calculate upwind speed (change in radial distance / delta time)
        print("Calculating upwind speed for Fly {}".format(index))
        instantaneous_radial_distance = np.linalg.norm(reference_fly_positions - origin, axis=1)
        instantaneous_upwind_speed = np.diff(instantaneous_radial_distance, axis=0) / np.diff(
            data["frame_times"], axis=0
        )
        instantaneous_upwind_speed = np.append(instantaneous_upwind_speed, np.nan)

        # calculate upwind heading (instantaneous motion heading - angular position in reference frame)
        print("Calculating upwind heading for Fly {}".format(index))
        instantaneous_upwind_heading = np.arctan2(
            np.diff(reference_fly_positions[:, 1], axis=0), np.diff(reference_fly_positions[:, 0], axis=0)
        ) - np.arctan2(reference_fly_positions[:-1, 1] - origin[1], reference_fly_positions[:-1, 0] - origin[0])
        instantaneous_upwind_heading = np.append(instantaneous_upwind_heading, np.nan)

        # calculate trial information
        print("Calculating trial information for Fly {}".format(index))
        start_arms = data["start_arms"]
        start_arms.append(relative_to_absolute_arm(data["chosen_arms"][-1], start_arms[-1], index))

        odor_vectors = data["odor_vectors"]
        odor_vectors.append([0.0, 0.0, 0.0])

        current_trials = np.int32(np.array(data["current_trial"]))
        current_arms = np.int32(np.array(data["current_arms"]))

        current_start_arms = np.array(start_arms)[current_trials]
        current_odor_vectors = np.array(odor_vectors)[current_trials]

        # calculate current odor
        print("Calculating current odor for Fly {}".format(index))
        current_odor = np.take_along_axis(current_odor_vectors, np.repeat(current_arms, 3).reshape(-1, 3), axis=1)[:, 0]

        current_odor_ = np.insert(current_odor, 0, 0)
        frame_times_ = np.insert(np.array(data["frame_times"]), 0, data["frame_times"][0])
        current_trial_ = np.int32(np.insert(np.array(data["current_trial"]), 0, data["current_trial"][0]))

        # calculate frames of odor encounter
        odor_encounter = current_odor_.copy()
        odor_encounter = np.append(odor_encounter, odor_encounter[-1] + 1)
        odor_encounter = np.diff(odor_encounter) != 0
        odor_encounter_ = odor_encounter.copy()
        odor_encounter_[0] = True

        # calculate time of odor encounter
        print("Calculating time of odor encounter for Fly {}".format(index))
        time_of_odor_encounter = frame_times_[odor_encounter_]

        # calculate trial number of each encounter
        print("Calculating trial number of each encounter for Fly {}".format(index))
        encounter_trial_number = current_trial_[odor_encounter]
        mask = encounter_trial_number < data["n_trials"]
        encounter_trial_number = encounter_trial_number[mask]

        # calculate odor at each encounter
        print("Calculating odor at each encounter for Fly {}".format(index))
        encounter_odor = current_odor_[odor_encounter]
        encounter_odor = encounter_odor[mask]

        # calculate duration of each encounter
        print("Calculating duration of each encounter for Fly {}".format(index))
        encounter_durations = np.diff(time_of_odor_encounter)
        encounter_durations = encounter_durations[mask]

        # calculate encounter decisions
        print("Calculating encounter decisions for Fly {}".format(index))
        encounter_decisions = np.int32(np.diff(np.append(encounter_trial_number, encounter_trial_number[-1] + 1)) != 0)

        # calculate encounter rewards
        print("Calculating encounter rewards for Fly {}".format(index))
        # print(len(data["reward_delivered"]), encounter_trial_number)
        encounter_rewards = np.int32(
            np.logical_and(
                np.array(data["reward_delivered"] + [0])[encounter_trial_number], encounter_decisions
            )  # add a zero for unfinished encounters
        )

        # calculate encounter start time
        print("Calculating encounter start time for Fly {}".format(index))
        encounter_start_time = time_of_odor_encounter[:-1][mask]

        # calculate odor residence time
        print("Calculating odor residence time for Fly {}".format(index))
        trials = np.int32(np.sort(np.unique(data["current_trial"])))
        trials = trials[trials > -1]
        odors = np.int32(np.sort(np.unique(current_odor)))

        trial_odor_residence_times = np.zeros((len(trials), len(odors)))

        for trial_number in trials:

            trial_current_odor = current_odor[np.array(data["current_trial"]) == trial_number]
            trial_frame_times = np.array(data["frame_times"])[np.array(data["current_trial"]) == trial_number]
            # append a new arm to the start and end of the trial
            trial_odor_encounter = trial_current_odor.copy()
            trial_odor_encounter = np.append(trial_odor_encounter, trial_odor_encounter[-1] + 1)
            # calculate every change in arm
            trial_odor_encounter = np.diff(trial_odor_encounter) != 0
            trial_odor_encounter_ = trial_odor_encounter.copy()
            trial_odor_encounter_[0] = True
            time_of_odor_encounter = trial_frame_times[trial_odor_encounter_]
            odor_at_encounter = trial_current_odor[trial_odor_encounter]
            # calculate duration of each arm
            odor_durations = np.diff(time_of_odor_encounter)
            # calculate residence time of each odor
            for odor in odors:
                try:
                    trial_odor_residence_times[trial_number, odor] = np.sum(odor_durations[odor_at_encounter == odor])
                except IndexError:
                    pass

        # calculate rewarded frames
        print("Calculating rewarded frames for Fly {}".format(index))
        trial_changed = np.diff(data["current_trial"] + [data["current_trial"][-1] + 1], axis=0)
        rewarded_frames = np.array(data["reward_delivered"] + [0])[np.int32(data["current_trial"])] * trial_changed

        # calculate trial oriented position
        print("Calculating trial oriented position for Fly {}".format(index))
        trial_oriented_position = np.array(
            [
                realign_lr(current_start_arms[i], reference_fly_positions[i], index)
                for i in range(len(data["frame_times"]))
            ]
        ).squeeze()

        # calculate odor oriented position
        print("Calculating odor oriented position for Fly {}".format(index))
        odor_oriented_position = np.array(
            [realign_odor(current_odor_vectors[i], reference_fly_positions[i]) for i in range(len(data["frame_times"]))]
        ).squeeze()

        print("All data calculated for Fly {}".format(index))

        # create the processed data dictionary by appending the new variables to the data dictionary
        processed_data = data
        processed_data["reference_fly_positions"] = reference_fly_positions.tolist()
        processed_data["instantaneous_speed"] = instantaneous_speed.tolist()
        processed_data["instantaneous_heading"] = instantaneous_heading.tolist()
        processed_data["instantaneous_upwind_speed"] = instantaneous_upwind_speed.tolist()
        processed_data["instantaneous_upwind_heading"] = instantaneous_upwind_heading.tolist()
        processed_data["current_odor"] = current_odor.tolist()
        processed_data["encounter_trial_number"] = encounter_trial_number.tolist()
        processed_data["encounter_odor"] = encounter_odor.tolist()
        processed_data["encounter_durations"] = encounter_durations.tolist()
        processed_data["encounter_decisions"] = encounter_decisions.tolist()
        processed_data["encounter_rewards"] = encounter_rewards.tolist()
        processed_data["encounter_start_time"] = encounter_start_time.tolist()
        processed_data["trial_odor_residence_times"] = trial_odor_residence_times.tolist()
        processed_data["rewarded_frames"] = rewarded_frames.tolist()
        processed_data["trial_oriented_position"] = trial_oriented_position.tolist()
        processed_data["odor_oriented_position"] = odor_oriented_position.tolist()

        # save the processed data
        with open(output_path, "w") as f:
            json.dump(processed_data, f)

    except Exception as e:
        print("Error in Fly {}: {}".format(index, e))


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, data_path=None, *args, **kwargs):
        """
        Creates the Data Processor window
        """
        super().__init__()
        self.setWindowTitle("SixteenY Data Processor")

        # store the data path
        self.data_path = data_path

        # create the main layout
        self.main_layout = QtWidgets.QGridLayout()

        # add a checkbox for Batch Mode
        self.batch_mode_checkbox = QtWidgets.QCheckBox("Batch Mode")
        self.batch_mode_checkbox.setChecked(False)
        self.main_layout.addWidget(self.batch_mode_checkbox, 0, 0, 1, 4)

        # add a label and an empty text box to enter the experiment folder along with a browse button
        self.experiment_folder_label = QtWidgets.QLabel("Experiment Folder:")
        self.experiment_folder_textbox = QtWidgets.QLineEdit()
        self.experiment_folder_textbox.setReadOnly(True)
        self.experiment_folder_browse_button = QtWidgets.QPushButton("Browse")
        self.experiment_folder_browse_button.clicked.connect(self.browse_for_experiment_folder)

        self.main_layout.addWidget(self.experiment_folder_label, 1, 0)
        self.main_layout.addWidget(self.experiment_folder_textbox, 1, 1, 1, 3)
        self.main_layout.addWidget(self.experiment_folder_browse_button, 1, 4)

        # add a checkbox to enable/disable parallel processing and a label and a text box to enter the number of threads
        self.parallel_processing_checkbox = QtWidgets.QCheckBox("Parallelize(?)")
        self.parallel_processing_checkbox.setChecked(True)

        self.parallel_processing_label = QtWidgets.QLabel("Number of Threads:")
        self.parallel_processing_textbox = QtWidgets.QLineEdit()
        self.parallel_processing_textbox.setText(str(multiprocessing.cpu_count()))
        self.parallel_processing_textbox.setValidator(QtGui.QIntValidator(1, multiprocessing.cpu_count()))

        self.main_layout.addWidget(self.parallel_processing_checkbox, 2, 0)
        self.main_layout.addWidget(self.parallel_processing_label, 2, 1, 1, 3)
        self.main_layout.addWidget(self.parallel_processing_textbox, 2, 4)

        # add a start button
        self.start_button = QtWidgets.QPushButton("Start")
        self.start_button.clicked.connect(self.start_data_processor)
        self.main_layout.addWidget(self.start_button, 3, 0, 1, 5)

        # add a label to show progress
        self.progress_label = QtWidgets.QLabel("Waiting to start...")
        self.main_layout.addWidget(self.progress_label, 4, 0, 1, 5)

        # create the central widget
        self.central_widget = QtWidgets.QWidget()
        self.central_widget.setLayout(self.main_layout)

        # set the central widget
        self.setCentralWidget(self.central_widget)

        self.show()

        # if a data path was provided, set the experiment folder text box to the data path
        if data_path is not None:
            self.experiment_folder_textbox.setText(data_path)

    def browse_for_experiment_folder(self):
        """
        Opens a folder dialog and sets the experiment folder text box to the selected folder
        """
        if self.batch_mode_checkbox.isChecked():
            dialog = QtWidgets.QFileDialog(self)
            dialog.setWindowTitle("Choose Directories")
            dialog.setOption(QtWidgets.QFileDialog.DontUseNativeDialog, True)
            dialog.setFileMode(QtWidgets.QFileDialog.DirectoryOnly)
            for view in dialog.findChildren((QtWidgets.QListView, QtWidgets.QTreeView)):
                if isinstance(view.model(), QtWidgets.QFileSystemModel):
                    view.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
            if dialog.exec_() == QtWidgets.QDialog.Accepted:
                folder_paths = dialog.selectedFiles()
                for folder_path in folder_paths:
                    if self.check_experiment_folder(folder_path):
                        continue
                    else:
                        QtWidgets.QMessageBox.warning(
                            self,
                            "Invalid Folder",
                            "The folder {} is not a valid experiment folder.".format(folder_path),
                        )

                self.experiment_folder_textbox.setText(";".join(folder_paths))
        else:
            folder_path = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Experiment Folder")
            if folder_path != "":
                # check if the folder is valid
                if self.check_experiment_folder(folder_path):
                    self.experiment_folder_textbox.setText(folder_path)
                    QtWidgets.QMessageBox.information(self, "Success", "Valid experiment folder found and selected")
                else:
                    QtWidgets.QMessageBox.warning(
                        self, "Invalid Folder", "The selected folder does not contain a valid experiment"
                    )

    def check_experiment_folder(self, folder_path):
        """
        Checks if the selected folder contains a valid experiment
        """
        # check if the folder contains a config file
        print(folder_path)
        config_file_path = os.path.join(folder_path, "config.yarena")
        if not os.path.isfile(config_file_path):
            # tell the user that the folder is not valid
            QtWidgets.QMessageBox.warning(
                self, "Invalid Folder", "The selected folder does not contain a valid configuration file"
            )
            return False
        return True

    def start_data_processor(self):
        """
        Starts the data processor
        """
        # get the experiment folder path
        experiment_folder_paths = self.experiment_folder_textbox.text()

        # split the paths if batch mode is enabled
        if self.batch_mode_checkbox.isChecked():
            experiment_folder_paths = experiment_folder_paths.split(";")
        else:
            experiment_folder_paths = [experiment_folder_paths]

        for experiment_folder_path in experiment_folder_paths:
            print("Processing experiment folder: {}".format(experiment_folder_path))

            # load mask labelled points
            self.progress_label.setText("Loading mask labelled points...")
            labelled_points = np.load(os.path.join(experiment_folder_path, "mask.npz"))["arm_keypoints"]

            # generate reference points
            self.progress_label.setText("Generating reference points...")
            reward_distance = 0.8

            origin = np.array([0.0, 0.0])
            arm_length = 1.0

            arm_width = arm_length * np.mean(
                [
                    np.mean(
                        [
                            euclidean_distance(labelled_points[i][0], labelled_points[i][1]),
                            euclidean_distance(labelled_points[i][1], labelled_points[i][2]),
                            euclidean_distance(labelled_points[i][2], labelled_points[i][0]),
                        ]
                    )
                    / np.mean(
                        [
                            euclidean_distance(
                                labelled_points[i][3],
                                (labelled_points[i][0] + labelled_points[i][1] + labelled_points[i][2]) / 3,
                            ),
                            euclidean_distance(
                                labelled_points[i][4],
                                (labelled_points[i][0] + labelled_points[i][1] + labelled_points[i][2]) / 3,
                            ),
                            euclidean_distance(
                                labelled_points[i][5],
                                (labelled_points[i][0] + labelled_points[i][1] + labelled_points[i][2]) / 3,
                            ),
                        ]
                    )
                    for i in range(len(labelled_points))
                ]
            )

            # get three corners of the central triangle
            k1 = origin + arm_width / np.sqrt(3) * np.array(
                [np.cos(np.pi / 2 + 2 * np.pi / 3), np.sin(np.pi / 2 + 2 * np.pi / 3)]
            )
            k2 = origin + arm_width / np.sqrt(3) * np.array(
                [np.cos(np.pi / 2 + 4 * np.pi / 3), np.sin(np.pi / 2 + 4 * np.pi / 3)]
            )
            k3 = origin + arm_width / np.sqrt(3) * np.array([np.cos(np.pi / 2), np.sin(np.pi / 2)])

            # get the endpoints of the arm
            k4 = origin + arm_length * np.array([np.cos(np.pi / 6 + 2 * np.pi / 3), np.sin(np.pi / 6 + 2 * np.pi / 3)])
            k5 = origin + arm_length * np.array([np.cos(np.pi / 6 + 4 * np.pi / 3), np.sin(np.pi / 6 + 4 * np.pi / 3)])
            k6 = origin + arm_length * np.array([np.cos(np.pi / 6), np.sin(np.pi / 6)])

            reference_points = np.array([k1, k2, k3, k4, k5, k6])

            # estimate the affine transformation between the reference points and the labelled points
            self.progress_label.setText("Estimating affine transformation...")
            estimated_transforms = []
            for i in range(len(labelled_points)):
                # get the points
                points = labelled_points[i]
                # get the affine transform
                transform = estimate_transform("affine", points, reference_points)
                # add to the list
                estimated_transforms.append(transform)

            # create the processed_data directory if it doesn't exist
            processed_folder_path = os.path.join(experiment_folder_path, "processed_data")
            if not os.path.isdir(processed_folder_path):
                os.mkdir(processed_folder_path)
            else:
                # delete the contents of the directory
                for file in os.listdir(processed_folder_path):
                    os.remove(os.path.join(processed_folder_path, file))

            # get data files and filter only the .ydata files
            data_files = os.listdir(os.path.join(experiment_folder_path, "data"))
            data_files = [file for file in data_files if file.endswith(".ydata")]

            # generate input and output file paths
            input_files = [os.path.join(experiment_folder_path, "data", file) for file in data_files]
            output_files = [os.path.join(processed_folder_path, file) for file in data_files]

            if not self.parallel_processing_checkbox.isChecked():
                # loop over the data and generate processed values
                for i in range(len(input_files)):
                    # get the input file path
                    input_file_path = input_files[i]

                    # get the output file path
                    output_file_path = output_files[i]

                    # process the data
                    self.progress_label.setText("Processing data...{}/{}".format(i + 1, len(input_files)))
                    QtCore.QCoreApplication.processEvents()
                    process_file(input_file_path, output_file_path, estimated_transforms, origin)
            else:
                self.progress_label.setText("Processing data...")
                QtCore.QCoreApplication.processEvents()

                # get the number of threads to use as the minimum of the thread count and the number of data files
                thread_count = min(int(self.parallel_processing_textbox.text()), len(input_files))

                # parallelize the processing
                Parallel(n_jobs=thread_count)(
                    delayed(process_file)(input_file_path, output_file_path, estimated_transforms, origin)
                    for input_file_path, output_file_path in zip(input_files, output_files)
                )

            # update the progress label
            self.progress_label.setText("Finished processing data!")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    if len(sys.argv) > 1:
        window = MainWindow(sys.argv[1])
    else:
        window = MainWindow()
    sys.exit(app.exec_())
