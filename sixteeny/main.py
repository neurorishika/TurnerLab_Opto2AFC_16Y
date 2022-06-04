from binascii import Incomplete
import numpy as np

# import cupy as cp
import matplotlib.pyplot as plt

from sixteeny.utils.tracker import ArenaTracker
from sixteeny.utils.experimenter import CSVExperimenter
from sixteeny.utils.camera import record_background, change_in_image, binarize

from sixteeny.controller.camera import SpinnakerCamera
from sixteeny.controller.odor import OdorValveController
from sixteeny.controller.led import LEDController
from sixteeny.controller.mfc import MFCController

import sys
import os
import json
import time
import datetime
import shutil

if __name__ == "__main__":
    # Get project directory and name from command line arguments
    if len(sys.argv) > 2:
        project_directory = sys.argv[1]
        experiment_name = sys.argv[2]
    else:
        # send error message
        print("Please specify experiment directory and name as command line arguments.")
        sys.exit(1)

    # ensure that project directory exists
    if project_directory[-1] != "/":
        project_directory += "/"
    if not os.path.isdir(project_directory):
        print("Project directory does not exist.")
        sys.exit(1)

    # ensure that experiment name directory exists
    if not os.path.isdir(project_directory + experiment_name):
        print("Experiment directory does not exist.")
        sys.exit(1)

    # load the rig configuration file
    if not os.path.isfile(project_directory + experiment_name + "/config.yarena"):
        print("Experiment config file does not exist.")
        sys.exit(1)

    with open(project_directory + experiment_name + "/config.yarena", "r") as f:
        rig_config = json.load(f)

    # find all *.yexperiment files in the folder
    experiment_files = [f for f in os.listdir(project_directory + experiment_name) if f.endswith(".yexperiment")]

    n_flies = len(experiment_files)
    fly_arenas = [int(experiment.split(".")[0].split("_")[1]) for experiment in experiment_files]

    # create video folder
    video_folder = project_directory + experiment_name + "/video/"
    if not os.path.isdir(video_folder):
        os.makedirs(video_folder)
    else:
        shutil.rmtree(video_folder)
        os.makedirs(video_folder)

    # import the skimage module
    if rig_config["enable_gpu_processing"]:
        import cupy as cp
        import cucim.skimage.morphology as skmorph
        import cucim.skimage.measure as skmeas
    else:
        import skimage.morphology as skmorph
        import skimage.measure as skmeas

    # Initialize controllers
    controllers = {}

    with SpinnakerCamera(
        index=rig_config["camera_index"],
        gpu_enabled=rig_config["enable_gpu_processing"],
        CAMERA_FORMAT=rig_config["pixel_format"],
        EXPOSURE_TIME=rig_config["exposure_time"],
        GAIN=rig_config["gain"],
        GAMMA=10,
        MAX_FRAME_RATE=rig_config["max_frame_rate"],
        record_video=rig_config["record_video"],
        video_output_path=video_folder,
        video_output_name=experiment_name + "_" + str(rig_config["camera_index"]),
        show_video=rig_config["show_video"],
        show_every_n=1,
        ffmpeg_path=rig_config["ffmpeg_path"],
    ) as camera, OdorValveController(minimum_delay=rig_config["minimum_message_delay"]) as odor, LEDController(
        ports=rig_config["com_ports"], baudrate=rig_config["baud_rate"], arena_panel_ids=rig_config["quadrant_ids"]
    ) as led, MFCController(
        com_port=rig_config["mfc_com_port"],
        device_ids=rig_config["mfc_device_ids"],
        default_flow_rate=rig_config["mfc_flow_rate"],
        default_gas_type=rig_config["mfc_gas_type"],
    ) as mfc:
        controllers["camera"] = camera
        controllers["odor"] = odor
        controllers["led"] = led
        controllers["mfc"] = mfc

        print("All controllers initialized.")

        # Start IR backlight
        led.turn_on_backlight(rig_config["ir_intensity"])
        print("IR backlight turned on.")

        # Check MFC values
        print("Checking MFC values...")

        n_observations = 10
        mfc_observations = np.zeros((n_observations, len(rig_config["mfc_device_ids"])))
        for i in range(n_observations):
            observed = False
            while not observed:
                try:
                    for j in range(len(rig_config["mfc_device_ids"])):
                        mfc_observations[i, j] = mfc.get_flow_rate(j)
                    observed = True
                except:
                    print("MFC communication error. Trying again...")
            time.sleep(1)

        error_margin = 0.1

        # get the average flow rate for each MFC
        average_flow_rates = np.mean(mfc_observations, axis=0)

        # check that the flow rates are within the expected range
        for i in range(len(rig_config["mfc_device_ids"])):
            if average_flow_rates[i] < rig_config["mfc_flow_rate"] * (1 - error_margin) or average_flow_rates[
                i
            ] > rig_config["mfc_flow_rate"] * (1 + error_margin):
                print(
                    "Flow rate for MFC "
                    + str(i)
                    + " is out of range. Expected "
                    + str(rig_config["mfc_flow_rate"])
                    + " mL/min, got "
                    + str(average_flow_rates[i])
                    + " mL/min."
                )
                sys.exit(1)

        print(
            "MFC values are within {}% error margin based on {} observations.".format(
                error_margin * 100, n_observations
            )
        )

        # flip all odor valves to air
        print("Flipping all odor valves to air...")
        for i in range(16):
            odor.publish(i, [0, 0, 0])
        print("All odor valves flipped to air.")

        # record the background image
        print("Recording background image for {} seconds...".format(rig_config["background_record_time"]))
        background, eff_fps, eff_duration, timestamp = record_background(
            time_to_record=rig_config["background_record_time"],
            camera=camera,
            gpu_enabled=rig_config["enable_gpu_processing"],
        )
        print("Background image recorded at {} fps for {} seconds").format(eff_fps, eff_duration)

        # show the background image
        if rig_config["enable_gpu_processing"]:
            plt.imshow(background.get())
        else:
            plt.imshow(background)
        plt.savefig(project_directory + experiment_name + "/background.png")
        plt.show()

        # Ask user to confirm background image
        background_confirmed = input("Is this the background image? (y/n) ")
        if background_confirmed == "y" or background_confirmed == "Y":
            print("Background image verified.")
        else:
            print("Background image capture failed. Exiting.")
            sys.exit(1)

        # load the mask
        if not os.path.isfile(rig_config["mask_file"]):
            print("Mask file does not exist. Exiting.")
            sys.exit(1)
        else:
            # copy the mask file to the experiment folder
            shutil.copy(rig_config["mask_file"], project_directory + experiment_name + "/mask.npy")
            # load the mask
            arm_mask, reward_mask, _ = np.load(rig_config["mask_file"])
            print("Mask loaded.")

        # overlay the mask on the background image
        if rig_config["enable_gpu_processing"]:
            plt.imshow(background.get())
        else:
            plt.imshow(background)
        plt.imshow(np.sum(arm_mask, axis=0) + np.sum(reward_mask, axis=0), alpha=0.5)
        plt.savefig(project_directory + experiment_name + "/mask_overlay.png")
        plt.show()

        # Ask user to confirm mask
        mask_confirmed = input("Is this the mask? (y/n) ")
        if mask_confirmed == "y" or mask_confirmed == "Y":
            print("Mask verified.")
        else:
            print("Mask was not verified. Exiting.")
            sys.exit(1)

        # determine the max value of the background image
        max_value = 255 if rig_config["pixel_format"] == "Mono8" else 65535

        # invert the background image
        background = max_value - background

        # create experimenters
        print("Creating experimenters...")
        experimenters = {}
        for i in experiment_files:
            # load experiment file as json
            with open(project_directory + experiment_name + "/" + i) as f:
                experiment_file = json.load(f)
            # get experimenter file
            if experiment_file["fly_experiment"].endswith(".csv"):
                experimenters[i] = CSVExperimenter(
                    project_directory + experiment_name + "/experiments/" + experiment_file["fly_experiment"]
                )
            else:
                print("Invalid experiment file. Exiting.")
                sys.exit(1)
        print("Experimenters created.")

        # start arena trackers
        trackers = {}
        for i in fly_arenas:
            tracker = ArenaTracker(i, experimenters[i], controllers)
            trackers[i] = tracker

        # start experiment loop
        print("Starting experiment...")
        experiment_ongoing = True

        camera.start()
        while experiment_ongoing:
            # get the current time
            current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

            # acquire a frame
            frame = camera.get_array()

            # invert the frame
            frame = max_value - frame

            # subtract the background image
            frame = change_in_image(frame, background, rig_config["enable_gpu_processing"])

            # binarize the frame
            frame = binarize(frame, rig_config["binarization_threshold"], rig_config["enable_gpu_processing"])

            # perform morphological operations
            frame = skmorph.binary_dilation(frame, selem=skmorph.disk(rig_config["dilation_radius"]))
            frame = skmorph.binary_erosion(frame, selem=skmorph.disk(rig_config["erosion_radius"]))

            # label the frame
            labels = skmeas.label(frame)

            # find all regions in the frame
            regions = skmeas.regionprops(labels)

            n_objects = len(regions)
            if n_objects == 0:
                print("No objects detected.")
                continue

            # enumerate all objects
            object_summary = np.zeros((n_objects, 6))
            for i in range(n_objects):
                # get position of the object
                position_x, position_y = regions[i].centroid
                # convert to integer indices
                position_x, position_y = int(position_x), int(position_y)
                # get the values at the mask indices
                arm_values = arm_mask[:, position_y, position_x]
                reward_values = reward_mask[:, position_y, position_x]
                # find the arena and arm and reward zone status
                arena = np.argmax(arm_values) // 3
                arm = np.argmax(arm_values) % 3
                area = regions[i].area
                in_reward_region = reward_mask[:, position_y, position_x].any and np.argmax(reward_values) == np.argmax(
                    arm_values
                )
                # add the object to the summary
                object_summary[i, :] = [position_x, position_y, arena, arm, area, in_reward_region]

            # loop over active arenas and find the largest object in each arena
            for i in fly_arenas:
                # find the largest object in the arena
                largest_object = np.argmax(object_summary[object_summary[:, 2] == i, 4])
                # get the position of the largest object
                position_x, position_y = (
                    object_summary[object_summary[:, 2] == i, 0][largest_object],
                    object_summary[object_summary[:, 2] == i, 1][largest_object],
                )
                # get the arm of the largest object
                arm = object_summary[object_summary[:, 2] == i, 3][largest_object]
                # get the reward region status of the largest object
                in_reward_region = object_summary[object_summary[:, 2] == i, 5][largest_object]
                # update the arena tracker
                trackers[i].update(arm, (position_x, position_y), in_reward_region)

            # run all LED stimuli
            led.run_accumulated_led_stimulus()

            # find all imcomplete arenas
            incomplete_arenas = [i for i in fly_arenas if not trackers[i].completed]

            if len(incomplete_arenas) == 0:
                # all arenas are complete
                print("All arenas complete.")
                experiment_ongoing = False

        # create data directory
        data_directory = project_directory + experiment_name + "/data/"
        if not os.path.exists(data_directory):
            os.makedirs(data_directory)
        else:
            # clear the data directory
            shutil.rmtree(data_directory)
            os.makedirs(data_directory)

        # save all tracker data
        for i in fly_arenas:
            trackers[i].save_data(project_directory + experiment_name + "/data/")

