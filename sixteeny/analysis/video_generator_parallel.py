from PyQt5 import QtWidgets, QtCore, QtGui

import numpy as np
import skimage
import skimage.io as io
from PIL import Image, ImageDraw, ImageFont
import os
import sys
import datetime
import json
import imageio
import time
from joblib import delayed, Parallel
import multiprocessing


def load_json(filename):
    with open(filename) as f:
        return json.load(f)


def parse_filename(filename):
    filename = filename.split(".")[0]
    year = int(filename.split("-")[0])
    month = int(filename.split("-")[1])
    day = int(filename.split("-")[2].split("_")[0])
    hour = int(filename.split("-")[2].split("_")[1])
    minute = int(filename.split("-")[3])
    second = int(filename.split("-")[4])
    millisecond = int(filename.split("-")[5])
    return datetime.datetime(year, month, day, hour, minute, second, millisecond).timestamp()


def get_timestamped_image(frame, duration):
    message = "T: +{:02d}h {:02d}m {:02d}s".format(
        int(duration // 3600), int(duration % 3600 // 60), int(duration % 60)
    )
    font = ImageFont.truetype("arial.ttf", 32)
    image = Image.new("RGBA", (frame.shape[1], frame.shape[0]), (0, 0, 0, 255))
    draw = ImageDraw.Draw(image)
    tw, th = draw.textsize(message, font=font)
    x, y = (frame.shape[1] - tw) / 2, (frame.shape[0] - th) / 2
    draw.text((x, y), message, (255, 255, 255), font=font)
    timestamp_image = np.clip(np.array(image), 0, 255)
    return np.max([timestamp_image, frame], axis=0)


def process_image(
    timestamp,
    image_file,
    trial_number,
    rewarded_frame,
    experiment_folder_path,
    data,
    arm_masks,
    combined_reward_mask,
    frame,
    start_time,
):

    # get the image
    image = io.imread(os.path.join(experiment_folder_path, "video", "processed", image_file))

    all_masks = []

    for i in range(len(data)):
        arm1mask = arm_masks[3 * i]
        arm2mask = arm_masks[3 * i + 1]
        arm3mask = arm_masks[3 * i + 2]
        trial_no = trial_number[i]

        if rewarded_frame[i] == 1:
            for mask in [arm1mask, arm2mask, arm3mask]:
                color_filter = np.repeat([np.repeat([[0, 1, 1]], mask.shape[1], axis=0)], mask.shape[0], axis=0)
                full_mask = np.repeat([np.int32(mask > 0) * 255], 3, axis=0).transpose(1, 2, 0)
                colored_mask = np.clip(full_mask * color_filter, 0, 255)
                all_masks.append(colored_mask)
        else:
            if trial_no < 0:
                arm1_odor, arm2_odor, arm3_odor = 0, 0, 0
            else:
                arm1_odor = data[i]["odor_vectors"][trial_no][0]
                arm2_odor = data[i]["odor_vectors"][trial_no][1]
                arm3_odor = data[i]["odor_vectors"][trial_no][2]
            for mask, odor in zip([arm1mask, arm2mask, arm3mask], [arm1_odor, arm2_odor, arm3_odor]):
                if odor == 0:
                    color_filter = np.repeat([np.repeat([[0, 0, 0]], mask.shape[1], axis=0)], mask.shape[0], axis=0)
                elif odor == 1:
                    color_filter = np.repeat([np.repeat([[0.5, 0, 1]], mask.shape[1], axis=0)], mask.shape[0], axis=0)
                elif odor == 2:
                    color_filter = np.repeat([np.repeat([[0, 0.5, 1]], mask.shape[1], axis=0)], mask.shape[0], axis=0)

                full_mask = np.repeat([np.int32(mask > 0) * 255], 3, axis=0).transpose(1, 2, 0)
                colored_mask = np.clip(full_mask * color_filter, 0, 255)
                all_masks.append(colored_mask)

    all_masks = np.array(all_masks)

    # combine the masks and the frame
    combined_mask = frame.copy()
    combined_mask[:, :, :3] = (frame[:, :, :3] - np.sum(all_masks, axis=0)) * combined_reward_mask

    # combine with the image
    processed_image = combined_mask.copy()
    processed_image[:, :, :3] = np.int32(skimage.util.invert(image[:, :, :3]) > 0) * processed_image[:, :, :3]

    # generate timestamp and add it to the image
    duration = timestamp - start_time
    processed_image = get_timestamped_image(processed_image, duration)
    return processed_image


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        """
        Creates the Video Generator Application
        """
        super().__init__()
        self.setWindowTitle("Video Generator")

        # create the main layout
        self.main_layout = QtWidgets.QGridLayout()

        # add a label and an empty text box to enter the experiment folder along with a browse button
        self.experiment_folder_label = QtWidgets.QLabel("Experiment Folder:")
        self.experiment_folder_textbox = QtWidgets.QLineEdit()
        self.experiment_folder_textbox.setReadOnly(True)
        self.experiment_folder_browse_button = QtWidgets.QPushButton("Browse")
        self.experiment_folder_browse_button.clicked.connect(self.browse_for_experiment_folder)

        self.main_layout.addWidget(self.experiment_folder_label, 0, 0)
        self.main_layout.addWidget(self.experiment_folder_textbox, 0, 1, 1, 3)
        self.main_layout.addWidget(self.experiment_folder_browse_button, 0, 4)

        # add a label and an empty text box to enter the speed multiplier
        self.speed_multiplier_label = QtWidgets.QLabel("Speed Multiplier:")
        self.speed_multiplier_textbox = QtWidgets.QLineEdit()
        self.speed_multiplier_textbox.setText("5")
        self.speed_multiplier_textbox.setValidator(QtGui.QDoubleValidator())
        self.main_layout.addWidget(self.speed_multiplier_label, 1, 0)
        self.main_layout.addWidget(self.speed_multiplier_textbox, 1, 1, 1, 4)

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
        self.start_button.clicked.connect(self.start_video_generation)
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

    def browse_for_experiment_folder(self):
        """
        Opens a folder dialog and sets the experiment folder text box to the selected folder
        """
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
        config_file_path = os.path.join(folder_path, "config.yarena")
        if not os.path.isfile(config_file_path):
            # tell the user that the folder is not valid
            QtWidgets.QMessageBox.warning(
                self, "Invalid Folder", "The selected folder does not contain a valid configuration file"
            )
            return False
        # check if the folder contains a video folder
        video_folder_path = os.path.join(folder_path, "video")
        if not os.path.isdir(video_folder_path):
            # tell the user that the folder is not valid
            QtWidgets.QMessageBox.warning(
                self, "Invalid Folder", "The selected folder does not contain a valid video folder"
            )
            return False
        # check if the video folder contains a processed folder
        processed_folder_path = os.path.join(video_folder_path, "processed")
        if not os.path.isdir(processed_folder_path):
            # tell the user that the folder is not valid
            QtWidgets.QMessageBox.warning(
                self, "Invalid Folder", "The selected folder does not contain a valid processed folder"
            )
            return False
        # check if the processed video folder contains .png files
        video_folder_contents = os.listdir(processed_folder_path)
        video_images = [file for file in video_folder_contents if file.endswith(".png")]
        if len(video_images) == 0:
            # tell the user that the folder is not valid
            QtWidgets.QMessageBox.warning(
                self, "Invalid Folder", "The selected folder does not contain any saved images"
            )
            return False
        # check if there is a processed data folder
        processed_data_folder_path = os.path.join(folder_path, "processed_data")
        if not os.path.isdir(processed_data_folder_path):
            # tell the user that the folder is not valid
            QtWidgets.QMessageBox.warning(
                self, "Invalid Folder", "The selected folder does not contain a valid processed data folder"
            )
            return False
        return True

    def start_video_generation(self):
        """
        Starts the video generation process
        """
        # get the experiment folder path
        experiment_folder_path = self.experiment_folder_textbox.text()

        # Get all image timestamps
        self.progress_label.setText("Getting image timestamps...")
        QtCore.QCoreApplication.processEvents()
        timestamps = [
            parse_filename(v)
            for v in filter(
                lambda v: ".png" in v, os.listdir(os.path.join(experiment_folder_path, "video", "processed"))
            )
        ]

        # Sort timestamps
        self.progress_label.setText("Sorting timestamps...")
        QtCore.QCoreApplication.processEvents()
        timestamps = list(sorted(timestamps))

        # find all the images in the video directory
        self.progress_label.setText("Finding images...")
        QtCore.QCoreApplication.processEvents()
        image_files = filter(
            lambda v: ".png" in v, os.listdir(os.path.join(experiment_folder_path, "video", "processed"))
        )

        # sort the images by their timestamp
        self.progress_label.setText("Sorting images...")
        QtCore.QCoreApplication.processEvents()
        image_files = list(sorted(image_files, key=lambda v: parse_filename(v)))

        # get data files
        self.progress_label.setText("Getting data files...")
        QtCore.QCoreApplication.processEvents()
        data_files = filter(lambda v: ".ydata" in v, os.listdir(os.path.join(experiment_folder_path, "processed_data")))
        data_files = list(sorted(data_files, key=lambda v: int(v.split(".")[0].split("_")[-1])))
        n_data_files = len(data_files)
        data = []

        # start progress update
        for i, data_file in enumerate(data_files):
            self.progress_label.setText("Loading data file {}/{}...".format(i + 1, n_data_files))
            QtCore.QCoreApplication.processEvents()
            data.append(load_json(os.path.join(experiment_folder_path, "processed_data", data_file)))

        # load the masks
        self.progress_label.setText("Loading masks...")
        QtCore.QCoreApplication.processEvents()
        masks = np.load(os.path.join(experiment_folder_path, "mask.npz"))
        arm_masks = masks["arm_masks"]
        arm_reward_masks = masks["arm_reward_masks"]

        # load the background
        self.progress_label.setText("Loading background...")
        QtCore.QCoreApplication.processEvents()
        background = skimage.io.imread(os.path.join(experiment_folder_path, "background.png"))

        # combine the masks
        self.progress_label.setText("Combining Background and Masks...")
        QtCore.QCoreApplication.processEvents()

        # create a combined mask
        combined_mask = np.repeat([np.int32(arm_masks.sum(axis=0) > 0) * 255], 3, axis=0).transpose(1, 2, 0)

        # create a combined reward mask
        combined_reward_mask = np.repeat([1 - 0.4 * np.int32(arm_reward_masks.sum(axis=0) > 0)], 3, axis=0).transpose(
            1, 2, 0
        )

        # create the combined background frame
        frame = background.copy()
        frame[:, :, :3] = np.clip(frame[:, :, :3] + combined_mask, 0, 255)

        # start the video generation process
        self.progress_label.setText("Generating video...")
        QtCore.QCoreApplication.processEvents()

        n_frames = len(image_files)
        frame_number = 0

        # calculate image capture rate
        capture_rate = 1.0 / np.mean(np.diff(timestamps))
        speed_up_multiplier = float(self.speed_multiplier_textbox.text())
        save_fps = int(capture_rate * speed_up_multiplier)

        # add a duplicate image and timestamp to the end of the list to make sure the video ends on a trial boundary
        image_files.append(image_files[-1])
        timestamps.append(timestamps[-1] + np.mean(np.diff(timestamps)))

        # calculate the trial numbers and rewarded trial
        self.progress_label.setText("Calculating trial numbers and rewarded trial...")
        QtCore.QCoreApplication.processEvents()
        trial_numbers = []
        for timestamp, image_file in zip(timestamps, image_files):
            arenas = []
            for i in range(len(data)):
                trial_no = (
                    np.argmax(timestamp <= np.array(data[i]["trial_end_times"]))
                    if timestamp < np.max(data[i]["trial_end_times"])
                    else -1
                )
                arenas.append(trial_no)
            trial_numbers.append(arenas)
        trial_numbers = np.array(trial_numbers)
        rewarded_trials = np.zeros_like(trial_numbers)[:-1, :]
        for i in range(rewarded_trials.shape[1]):
            rewarded_trials[:, i] = np.array(data[i]["reward_delivered"])[np.int32(trial_numbers[:, i])][
                :-1
            ] * np.int32(np.diff(trial_numbers[:, i]) != 0)
        rewarded_trials = rewarded_trials.astype(np.int32)[:-1, :]

        # get the timestamp of the first image
        start_time = timestamps[0]

        if not self.parallel_processing_checkbox.isChecked():

            # loop through the images
            processed_images = []
            times = []
            for timestamp, image_file, trial_number, rewarded_trial in zip(
                timestamps, image_files, trial_numbers, rewarded_trials
            ):
                # estimate the time left
                times.append(time.time())
                if len(times) > 1:
                    time_per_iteration = np.mean(np.diff(times))
                    time_left = (n_frames - frame_number) * time_per_iteration
                    time_per_iteration = "{:.2f} s".format(time_per_iteration)
                    time_left = "{:02d}h {:02d}m {:02d}s".format(
                        int(time_left // 3600), int((time_left % 3600) // 60), int(time_left % 60)
                    )
                else:
                    time_per_iteration = "- s"
                    time_left = "-h -m -s"

                # update the progress label
                self.progress_label.setText(
                    "Generating frame {}/{}. {} per iteration. {} left.".format(
                        frame_number + 1, n_frames, time_per_iteration, time_left
                    )
                )
                frame_number += 1
                QtCore.QCoreApplication.processEvents()

                processed_image = process_image(
                    timestamp,
                    image_file,
                    trial_number,
                    rewarded_trial,
                    experiment_folder_path,
                    data,
                    arm_masks,
                    combined_reward_mask,
                    frame,
                    start_time,
                )
                processed_images.append(processed_image)

        else:
            # get the thread count as the minimum of the number of images and the number of threads
            thread_count = min(len(image_files), int(self.parallel_processing_textbox.text()))

            # parallelize the processing using Parallel and delayed
            self.progress_label.setText("Generating frames using {} threads...".format(thread_count))
            QtCore.QCoreApplication.processEvents()

            processed_images = Parallel(n_jobs=thread_count)(
                delayed(process_image)(
                    timestamp,
                    image_file,
                    trial_number,
                    rewarded_trial,
                    experiment_folder_path,
                    data,
                    arm_masks,
                    combined_reward_mask,
                    frame,
                    start_time,
                )
                for timestamp, image_file, trial_number, rewarded_trial in zip(
                    timestamps, image_files, trial_numbers, rewarded_trials
                )
            )

        # save the video as a lossless mp4
        self.progress_label.setText("Saving video...")
        imageio.mimsave(os.path.join(experiment_folder_path, "processed_video.mp4"), processed_images, fps=save_fps)

        self.progress_label.setText("Video generation complete!")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())

