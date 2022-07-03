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
    second = int(filename.split("-")[4].split("_")[0])  # needs to be fixed!
    # millisecond = int(filename.split("-")[5])
    return datetime.datetime(year, month, day, hour, minute, second).timestamp()


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
        self.main_layout.addWidget(self.speed_multiplier_textbox, 1, 1, 1, 3)

        # add a start button
        self.start_button = QtWidgets.QPushButton("Start")
        self.start_button.clicked.connect(self.start_video_generation)
        self.main_layout.addWidget(self.start_button, 1, 4)

        # add a label to show progress
        self.progress_label = QtWidgets.QLabel("Waiting to start...")
        self.main_layout.addWidget(self.progress_label, 2, 0, 1, 5)

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
        return True

    def start_video_generation(self):
        """
        Starts the video generation process
        """
        # get the experiment folder path
        experiment_folder_path = self.experiment_folder_textbox.text()

        # Get all image timestamps
        self.progress_label.setText("Getting image timestamps...")
        timestamps = [
            parse_filename(v)
            for v in filter(
                lambda v: ".png" in v, os.listdir(os.path.join(experiment_folder_path, "video", "processed"))
            )
        ]

        # Sort timestamps
        self.progress_label.setText("Sorting timestamps...")
        timestamps = list(sorted(timestamps))

        # find all the images in the video directory
        self.progress_label.setText("Finding images...")
        image_files = filter(
            lambda v: ".png" in v, os.listdir(os.path.join(experiment_folder_path, "video", "processed"))
        )

        # sort the images by their timestamp
        self.progress_label.setText("Sorting images...")
        image_files = list(sorted(image_files, key=lambda v: parse_filename(v)))

        # get data files
        self.progress_label.setText("Getting data files...")
        data_files = filter(lambda v: ".ydatanew" in v, os.listdir(os.path.join(experiment_folder_path, "data")))
        data_files = list(sorted(data_files, key=lambda v: int(v.split(".")[0].split("_")[-1])))
        n_data_files = len(data_files)
        data = []

        # start progress bar
        for i, data_file in enumerate(data_files):
            self.progress_label.setText("Loading data file {}/{}...".format(i + 1, n_data_files))
            data.append(load_json(os.path.join(experiment_folder_path, "data", data_file)))

        # load the masks
        self.progress_label.setText("Loading masks...")
        masks = np.load(os.path.join(experiment_folder_path, "mask.npz"))
        arm_masks = masks["arm_masks"]
        arm_reward_masks = masks["arm_reward_masks"]

        # load the background
        self.progress_label.setText("Loading background...")
        background = skimage.io.imread(os.path.join(experiment_folder_path, "background.png"))

        # combine the masks
        self.progress_label.setText("Combining Background and Masks...")

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

        n_frames = len(image_files)
        frame_number = 0

        current_trial = [0] * len(data)
        processed_images = []

        times = []

        # calculate image capture rate
        capture_rate = 1.0 / np.mean(np.diff(timestamps))
        speed_up_multiplier = float(self.speed_multiplier_textbox.text())
        save_fps = int(capture_rate * speed_up_multiplier)
        # print("Capture rate: {}".format(capture_rate), "Save FPS: {}".format(save_fps))

        # add a duplicate image and timestamp to the end of the list to make sure the video ends on a trial boundary
        image_files.append(image_files[-1])
        timestamps.append(timestamps[-1] + np.mean(np.diff(timestamps)))

        # get the timestamp of the first image
        start_time = timestamps[0]

        for timestamp, image_file in zip(timestamps, image_files):

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

            self.progress_label.setText(
                "Generating frame {}/{}. {} per iteration. {} left.".format(
                    frame_number + 1, n_frames, time_per_iteration, time_left
                )
            )
            frame_number += 1
            QtCore.QCoreApplication.processEvents()

            # get the image
            image = io.imread(os.path.join(experiment_folder_path, "video", "processed", image_file))

            all_masks = []

            for i in range(len(data)):
                arm1mask = arm_masks[3 * i]
                arm2mask = arm_masks[3 * i + 1]
                arm3mask = arm_masks[3 * i + 2]
                trial_no = np.argmax(timestamp < np.array(data[i]["trial_start_times"])) - 1

                if trial_no > current_trial[i] and data[i]["reward_delivered"][trial_no - 1] == 1.0:
                    for mask in [arm1mask, arm2mask, arm3mask]:
                        color_filter = np.repeat([np.repeat([[0, 1, 1]], mask.shape[1], axis=0)], mask.shape[0], axis=0)
                        full_mask = np.repeat([np.int32(mask > 0) * 255], 3, axis=0).transpose(1, 2, 0)
                        colored_mask = np.clip(full_mask * color_filter, 0, 255)
                        all_masks.append(colored_mask)
                    current_trial[i] = trial_no

                else:
                    if trial_no < 0:
                        arm1_odor, arm2_odor, arm3_odor = 0, 0, 0
                    else:
                        arm1_odor = data[i]["odor_vectors"][trial_no][0]
                        arm2_odor = data[i]["odor_vectors"][trial_no][1]
                        arm3_odor = data[i]["odor_vectors"][trial_no][2]
                    for mask, odor in zip([arm1mask, arm2mask, arm3mask], [arm1_odor, arm2_odor, arm3_odor]):
                        if odor == 0:
                            color_filter = np.repeat(
                                [np.repeat([[0, 0, 0]], mask.shape[1], axis=0)], mask.shape[0], axis=0
                            )
                        elif odor == 1:
                            color_filter = np.repeat(
                                [np.repeat([[0.5, 0, 1]], mask.shape[1], axis=0)], mask.shape[0], axis=0
                            )
                        elif odor == 2:
                            color_filter = np.repeat(
                                [np.repeat([[0, 0.5, 1]], mask.shape[1], axis=0)], mask.shape[0], axis=0
                            )

                        full_mask = np.repeat([np.int32(mask > 0) * 255], 3, axis=0).transpose(1, 2, 0)
                        colored_mask = np.clip(full_mask * color_filter, 0, 255)
                        all_masks.append(colored_mask)
                    if trial_no > current_trial[i]:
                        current_trial[i] = trial_no
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

            processed_images.append(processed_image)

        # save the video as a lossless mp4
        self.progress_label.setText("Saving video...")
        imageio.mimsave(os.path.join(experiment_folder_path, "processed_video.mp4"), processed_images, fps=save_fps)

        self.progress_label.setText("Video generation complete!")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())

