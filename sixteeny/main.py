import subprocess
import sys

import numpy as np
import cupy as cp
import matplotlib.pyplot as plt

from sixteeny.utils.tracker import ArenaTracker
from sixteeny.utils.experimenter import CSVExperimenter, DeterministicFiniteStateExperimenter, DeterministicMultilevelExperimenter
from sixteeny.utils.camera import record_background, change_in_image, binarize, combine_binarized_images
from sixteeny.utils.emailer import Emailer
from sixteeny.utils.printer import Printer

from sixteeny.controller.camera import SpinnakerCamera
from sixteeny.controller.odor import OdorValveController
from sixteeny.controller.led import LEDController
from sixteeny.controller.mfc import MFCController

import os
import json
import time
import datetime
import shutil
import threading
import queue

frame_write_count = 0

def async_video_writer(save_queue, gpu, pre_acquired=False):
    """
    Save a frame queue to .png files asynchronously.
    """
    global frame_write_count
    while True:
        image, filename = save_queue.get()
        if not pre_acquired and gpu:
            image = image.get()
        if image is None:
            break
        else:
            # save image to disk
            plt.imsave(filename, image, cmap="gray")
            frame_write_count += 1
            if frame_write_count % 100 == 0:
                print("Saved {} frames. {} frames left in queue.".format(frame_write_count, save_queue.qsize() - 1))
            save_queue.task_done()


def async_email_sender(email_queue, emailer):
    """
    Send an email queue asynchronously.
    """
    while True:
        email_text, email_subject = email_queue.get()
        if email_text is None:
            break
        else:
            # send email
            emailer.SendMessage(email_subject, email_text, email_text)
            email_queue.task_done()

def async_reminder(frequency, email_queue):
    while True:
        time.sleep(frequency)
        email_queue.put(
            (
                "Warning: The experiment is still complete but the air valves are still on. Please turn them off and proceed to the next experiment to prevent damage to the arena.",
                "URGENT: AIR SUPPLY IS ON AFTER EXPERIMENT HAS ENDED",
            )
        )

if __name__ == "__main__":

    start_string = """
====================================================================
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    ██╗ ██████╗██╗   ██╗     ███╗   ███╗ █████╗ ███████╗███████╗
   ███║██╔════╝╚██╗ ██╔╝     ████╗ ████║██╔══██╗╚══███╔╝██╔════╝
   ╚██║███████╗ ╚████╔╝█████╗██╔████╔██║███████║  ███╔╝ █████╗  
    ██║██╔═══██╗ ╚██╔╝ ╚════╝██║╚██╔╝██║██╔══██║ ███╔╝  ██╔══╝  
    ██║╚██████╔╝  ██║        ██║ ╚═╝ ██║██║  ██║███████╗███████╗
    ╚═╝ ╚═════╝   ╚═╝        ╚═╝     ╚═╝╚═╝  ╚═╝╚══════╝╚══════╝
    Developed by:
    - Rishika Mohanta, Turner Lab, Janelia Research Campus
    
            " May the flies be ever in your favor "
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
====================================================================
    """
    print(start_string)

    # Get project directory and name from command line arguments
    if len(sys.argv) > 2:
        project_directory = sys.argv[1]
        experiment_name = sys.argv[2]
        print("Project directory: " + project_directory)
        print("Experiment name: " + experiment_name)
    else:
        # send error message
        print("Please specify experiment directory and name as command line arguments.")
        sys.exit(1)

    print()

    # ensure that project directory exists
    if project_directory[-1] != "/":
        project_directory += "/"
    if not os.path.isdir(project_directory):
        print("Project directory does not exist.")
        sys.exit(1)
    else:
        print("Project directory found.")

    # ensure that experiment name directory exists
    if not os.path.isdir(project_directory + experiment_name):
        print("Experiment directory does not exist.")
        sys.exit(1)
    else:
        print("Experiment directory found.")

    print()

    # create a log file and add it to a Printer
    printer = Printer(project_directory + experiment_name + "/log.txt")
    printer.print("Project directory: " + project_directory, dont_print_to_console=True)
    printer.print("Experiment name: " + experiment_name, dont_print_to_console=True)
    printer.print("", dont_print_to_console=True)
    printer.print("Project directory found.", dont_print_to_console=True)
    printer.print("Experiment directory found.", dont_print_to_console=True)

    # load the rig configuration file
    if not os.path.isfile(project_directory + experiment_name + "/config.yarena"):
        printer.print("Experiment config file does not exist.")
        sys.exit(1)
    else:
        printer.print("Experiment config file found.")

    with open(project_directory + experiment_name + "/config.yarena", "r") as f:
        rig_config = json.load(f)
    printer.print("Rig config loaded.")

    # find all *.yexperiment files in the folder
    experiment_files = [f for f in os.listdir(project_directory + experiment_name) if f.endswith(".yexperiment")]
    printer.print("Found " + str(len(experiment_files)) + " experiment files.")

    n_flies = len(experiment_files)
    fly_arenas = [int(experiment.split(".")[0].split("_")[1]) for experiment in experiment_files]

    # create video folder
    video_folder = project_directory + experiment_name + "/video/"
    # if the folder already exists, delete it
    if os.path.isdir(video_folder):
        shutil.rmtree(video_folder)
    # create the folder
    os.mkdir(video_folder)
    printer.print("Video folder created.")
    printer.print("")

    # ask the user if they have turned on the air supply
    while True:
        air_supply = input("Have you turned on the MFCs and air supply? (y/n) ")
        if air_supply == "y" or air_supply == "Y":
            break
        elif air_supply == "n" or air_supply == "N":
            printer.print("Please turn on the air supply and try again.")
        else:
            printer.print("Please enter either 'y' or 'n'.")

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

    printer.print("Initializing controllers.")

    with SpinnakerCamera(
        index=rig_config["camera_index"],
        gpu_enabled=rig_config["enable_gpu_processing"],
        CAMERA_FORMAT=rig_config["pixel_format"],
        EXPOSURE_TIME=rig_config["exposure_time"],
        GAIN=rig_config["gain"],
        GAMMA=rig_config["gamma"],
        record_video=rig_config["record_video"],
        video_output_path=video_folder,
        video_output_name=experiment_name[5:] + "_" + str(rig_config["camera_index"]),
        show_video=rig_config["live_stream"],
        show_every_n=1,
        lossless=rig_config["lossless_compression"],
        fast_mode=rig_config["fast_mode"],
    ) as camera, OdorValveController(minimum_delay=rig_config["minimum_message_delay"] / 1000) as odor, LEDController(
        ports=rig_config["com_ports"], baudrate=rig_config["baud_rate"], arena_panel_ids=rig_config["quadrant_ids"],
        irgb_scaling_factors=rig_config["led_scaling_factors"],printer=printer
    ) as led, MFCController(
        com_port=rig_config["mfc_com_port"],
        device_ids=rig_config[
            "mfc_device_ids"
        ],  # [i for n,i in enumerate(rig_config["mfc_device_ids"]) if n in fly_arenas],
        default_flow_rate=0.0,
    ) as mfc:
        controllers["camera"] = camera
        controllers["odor"] = odor
        controllers["led"] = led
        controllers["mfc"] = mfc

        printer.print("All controllers initialized.\n")

        # turn on MFCs
        for i in range(16):
            controllers["mfc"].set_flow_rate(i, rig_config["mfc_flow_rate"])
        printer.print("MFCs turned on.\n")

        # Start IR backlight
        led.turn_on_backlight(rig_config["ir_intensity"])
        printer.print("IR backlight turned on.")

        # Check MFC values
        printer.print("Checking MFC values...")

        keep_trying = True
        while keep_trying:
            n_observations = 10
            mfc_observations = np.zeros((n_observations, 16))
            for i in range(n_observations):
                observed = False
                while not observed:
                    try:
                        for j in range(16):
                            mfc_observations[i, j] = mfc.get_flow_rate(j)
                        observed = True
                    except:
                        printer.print("MFC communication error for device " + str(j) + ".")
                time.sleep(1)

            error_margin = 0.1

            # get the average flow rate for each MFC
            average_flow_rates = np.mean(mfc_observations, axis=0)

            # check that the flow rates are within the expected range
            for i, n in enumerate(fly_arenas):
                if average_flow_rates[i] < rig_config["mfc_flow_rate"] * (1 - error_margin) or average_flow_rates[
                    i
                ] > rig_config["mfc_flow_rate"] * (1 + error_margin):
                    printer.print(
                        "Flow rate for MFC "
                        + str(n)
                        + " is out of range. Expected "
                        + str(rig_config["mfc_flow_rate"])
                        + " mL/min, got "
                        + str(average_flow_rates[i])
                        + " mL/min."
                    )
                    keep_trying = True
                    retry = input("Retry? (y/n) ")
                    if retry == "n":
                        printer.print("Exiting.")
                        sys.exit(1)
                    else:
                        break
                else:
                    keep_trying = False

        printer.print(
            "Flow rates are:\n {} mL/min.".format(" ml/min\n ".join(["{:0.2f}".format(x) for x in average_flow_rates]))
        )
        printer.print(
            "MFC values are within {}% error margin based on {} observations.".format(
                error_margin * 100, n_observations
            )
        )

        # flip all odor valves to air
        printer.print("Flipping all odor valves to air...")
        for i in range(16):
            odor.publish(i, [0, 0, 0])
        printer.print("All odor valves flipped to air.")

        retry = True
        while retry:
            # record the background image
            printer.print("Recording background image for {} seconds...".format(rig_config["background_calculation_time"]))
            background, eff_fps, eff_duration, timestamp = record_background(
                time_to_record=rig_config["background_calculation_time"],
                camera=camera,
                gpu_enabled=rig_config["enable_gpu_processing"],
            )
            printer.print("Background image recorded at {} fps for {} seconds".format(eff_fps, eff_duration))

            # show the background image
            if rig_config["enable_gpu_processing"]:
                plt.imshow(background.get())
                plt.imsave(project_directory + experiment_name + "/background.png", background.get(), cmap="gray")
            else:
                plt.imshow(background)
                plt.imsave(project_directory + experiment_name + "/background.png", background, cmap="gray")
            plt.show()

            # Ask user to confirm background image and wait for correct input
            while True:
                background_confirmed = input("Is this the background image? (y/n) ")
                if background_confirmed == "y" or background_confirmed == "Y":
                    printer.print("Background image verified.")
                    retry = False
                    break
                elif background_confirmed == "n" or background_confirmed == "N":
                    printer.print("Background image capture failed. Retry?")
                    while True:
                        entry = input("Retry? (y/n) ")
                        if entry == "y" or entry == "Y":
                            retry = True
                            break
                        elif entry == "n" or entry == "N":
                            printer.print("Exiting.")
                            sys.exit(1)
                        else:
                            printer.print("Invalid input. Enter 'y' or 'n'.")
                else:
                    printer.print("Invalid input. Enter 'y' or 'n'.")
                    continue
                if retry:
                    break

        # load the mask
        if not os.path.isfile(rig_config["mask_file"]):
            printer.print("Mask file does not exist.")
            # ask if the user wants to create a new mask
            create_new_mask = input("Create new mask? (y/n) ")
            if create_new_mask == "y":
                # run mask_designer.py with the background image as the argument and wait for it to finish
                printer.print(
                    "Running mask designer...\nPlease make sure to save the mask file as mask.npz in the experiment directory."
                )
                subprocess.call(
                    [
                        "python",
                        "sixteeny/gui/16Y_mask_designer.py",
                        project_directory + experiment_name + "/background.png",
                    ]
                )
                printer.print("Mask designer complete.")
                # load the mask
                temp = np.load(project_directory + experiment_name + "/mask.npz", allow_pickle=True)
                arm_mask = temp["arm_masks"]
                reward_mask = temp["arm_reward_masks"]
                printer.print("Mask loaded.")
            else:
                printer.print("Exiting.")
                sys.exit(1)
        else:
            # copy the mask file to the experiment folder
            shutil.copy(rig_config["mask_file"], project_directory + experiment_name + "/mask.npz")
            # load the mask
            temp = np.load(project_directory + experiment_name + "/mask.npz", allow_pickle=True)
            arm_mask = temp["arm_masks"]
            reward_mask = temp["arm_reward_masks"]
            combined_mask = temp["combined_mask"]
            printer.print("Mask loaded.")

        # overlay the mask on the background image
        if rig_config["enable_gpu_processing"]:
            plt.imshow(background.get())
        else:
            plt.imshow(background)
        plt.imshow(np.sum(arm_mask, axis=0) + np.sum(reward_mask, axis=0), alpha=0.5)
        plt.imsave(
            project_directory + experiment_name + "/mask.png",
            np.sum(arm_mask, axis=0) + np.sum(reward_mask, axis=0),
            cmap="gray",
        )
        plt.show()

        # Ask user to confirm mask and wait for correct response
        while True:
            mask_confirmed = input("Is this the mask? (y/n) ")
            if mask_confirmed == "y" or mask_confirmed == "Y":
                printer.print("Mask verified.")
                break
            elif mask_confirmed == "n" or mask_confirmed == "N":
                printer.print("Mask capture failed. Exiting.")
                sys.exit(1)
            else:
                printer.print("Invalid input. Enter 'y' or 'n'.")
                continue

        # convert the combined mask to a correct format
        if rig_config["enable_gpu_processing"]:
            combined_mask = cp.array(combined_mask)
        else:
            combined_mask = np.array(combined_mask)

        # determine the max value of the background image
        max_value = 255 if rig_config["pixel_format"] == "Mono8" else 65535

        # invert the background image
        background = max_value - background

        # create experimenters
        printer.print("Creating experimenters...")
        experimenters = {}
        for i, n in zip(experiment_files, fly_arenas):
            # load experiment file as json
            with open(project_directory + experiment_name + "/" + i) as f:
                experiment_file = json.load(f)
            # get experimenter file
            if experiment_file["fly_experiment"].endswith(".csv"):
                experimenters[n] = CSVExperimenter(
                    project_directory + experiment_name + "/experiments/" + experiment_file["fly_experiment"]
                )
            elif experiment_file["fly_experiment"].endswith(".yfse"):
                experimenters[n] = DeterministicFiniteStateExperimenter(
                    project_directory + experiment_name + "/experiments/" + experiment_file["fly_experiment"]
                )
            elif experiment_file["fly_experiment"].endswith(".ymle"):
                experimenters[n] = DeterministicMultilevelExperimenter(
                    project_directory + experiment_name + "/experiments/" + experiment_file["fly_experiment"]
                )
            else:
                printer.print("Invalid experiment file. Exiting.")
                sys.exit(1)
        printer.print("Experimenters created.")

        # start arena trackers
        trackers = {}
        for i in fly_arenas:
            tracker = ArenaTracker(i, experimenters[i], controllers)
            trackers[i] = tracker

        # ask user if they want to start the experiment in debug mode
        while True:
            debug_mode = input("Start in debug mode? (y/n) ")
            if debug_mode == "y" or debug_mode == "Y":
                debug_mode = True
                printer.print("Debug mode enabled.")
                break
            elif debug_mode == "n" or debug_mode == "N":
                debug_mode = False
                printer.print("Debug mode disabled.")
                break
            else:
                printer.print("Invalid input. Enter 'y' or 'n'.")
                continue

        # start experiment loop
        printer.print("Starting experiment...")
        experiment_ongoing = True

        # if live stream is enabled, create a new save queue and thread
        if rig_config["live_stream"]:
            # check if there is a processed folder in the video directory
            if not os.path.isdir(project_directory + experiment_name + "/video/processed"):
                os.mkdir(project_directory + experiment_name + "/video/processed")
            else:
                # delete all files in the processed folder
                for f in os.listdir(project_directory + experiment_name + "/video/processed"):
                    os.remove(project_directory + experiment_name + "/video/processed/" + f)
            # create a new save queue
            save_queue = queue.Queue()
            save_thread = threading.Thread(
                target=async_video_writer, args=(save_queue, rig_config["enable_gpu_processing"], True)
            )
            save_thread.start()
        
        # if email notification is enabled, create a new email queue and thread
        if rig_config["email_notifications"]:
            # create emailer
            printer.print("Creating emailer...")
            emailer = Emailer(recipients=rig_config["email_addresses"], printer=printer)
            printer.print("Emailer created.")

            # create an email queue
            email_queue = queue.Queue()
            email_thread = threading.Thread(target=async_email_sender, args=(email_queue, emailer))
            email_thread.start()

            # email verifiers
            halfway_email_sent = False
            last_fly_email_sent = False

        # start camera
        camera.start()
        frame_number = 0

        started = False
        wait_period = 10  # seconds
        maximum_time = 2.5 * 60 * 60  # seconds (3 hours)
        tracking_start_time = time.time()

        # send email
        if rig_config["email_notifications"]:
            email_queue.put(
                            (
                                "The experiment has started successfully. You will be notified when the experiment is complete.",
                                "Experiment Started Successfully at " + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f"),
                            )
                        )

        while experiment_ongoing:
            try:
                # get the current time
                current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")

                # acquire a frame
                frame = camera.get_array()

                # invert the frame
                frame = max_value - frame

                # subtract the background image
                frame = change_in_image(
                    frame, background, rig_config["threshold_type"] == "relative", rig_config["enable_gpu_processing"]
                )

                # binarize the frame
                if rig_config["threshold_type"] == "relative":
                    frame = binarize(
                        frame, rig_config["binarization_threshold_relative"], rig_config["enable_gpu_processing"]
                    )
                else:
                    frame = binarize(
                        frame, rig_config["binarization_threshold_absolute"], rig_config["enable_gpu_processing"]
                    )

                # # perform morphological operations

                # apply binary closing
                frame = skmorph.binary_closing(frame, skmorph.disk(rig_config["closing_radius"]))

                # filter the frame using the combined mask
                frame = combine_binarized_images(frame, combined_mask, "and", rig_config["enable_gpu_processing"])

                # label the frame
                labels = skmeas.label(frame)

                # find all regions in the frame
                regions = skmeas.regionprops(labels)

                n_objects = len(regions)

                # enumerate all objects
                object_summary = np.ones((n_objects, 6)) * np.nan

                for i in range(n_objects):
                    # get position of the object
                    position_x, position_y = regions[i].centroid
                    # convert to integer indices
                    position_x, position_y = int(position_x), int(position_y)
                    # get the values at the mask indices
                    arm_values = arm_mask[:, position_x, position_y]
                    reward_values = reward_mask[:, position_x, position_y]
                    # check if the object is in any of the arenas

                    if np.any(arm_values > 0):
                        # find the arena and arm and reward zone status
                        # print(arm_values)
                        arena = np.argmax(arm_values) // 3
                        arm = np.argmax(arm_values) % 3
                        area = regions[i].area
                        in_reward_region = reward_mask[:, position_x, position_y].any() and np.argmax(
                            reward_values
                        ) == np.argmax(arm_values)
                        # add the object to the summary
                        area = area.get() if rig_config["enable_gpu_processing"] else area
                        object_summary[i, :] = [position_x, position_y, arena, arm, area, float(in_reward_region)]

                # remove all rows with NaN values
                object_summary = object_summary[~np.isnan(object_summary).any(axis=1)]

                if debug_mode and started:
                    printer.print("T: " + str(current_time), end="\t")

                # loop over active arenas and find the largest object in each arena
                detected = []
                rewarded = []
                for i in fly_arenas:

                    # see if any objects are in the arena
                    if not np.any(object_summary[:, 2] == i):
                        if debug_mode and started:
                            printer.print("NA", end="\t")
                        continue
                    else:
                        detected.append(i)

                    # find the largest object in the arena
                    largest_object = np.argmax(object_summary[object_summary[:, 2] == i, 4])

                    # get the position of the largest object
                    position_x, position_y = (
                        object_summary[object_summary[:, 2] == i, 0][largest_object],
                        object_summary[object_summary[:, 2] == i, 1][largest_object],
                    )

                    # get the arm of the largest object
                    arm = int(object_summary[object_summary[:, 2] == i, 3][largest_object])
                    # get the reward region status of the largest object
                    in_reward_region = bool(object_summary[object_summary[:, 2] == i, 5][largest_object])

                    # update the arena tracker if wait period has passed

                    if started:

                        if debug_mode:
                            printer.print(
                                str(trackers[i].trial_count + 1)
                                + ","
                                + str(arm)
                                + ","
                                + str(1 if in_reward_region else 0),
                                end="\t",
                            )

                        reward = trackers[i].update(arm, (position_x, position_y), in_reward_region)

                        # if the frame was rewarded, add the arena to the list of rewarded arenas
                        if reward:
                            rewarded.append(i)

                if debug_mode and started:
                    printer.print("")

                # if live stream is enabled and any fly was detected, save the frame
                if rig_config["live_stream"] and len(detected) > 0 and frame_number % 10 == 0 and started:
                    save_queue.put(
                        (
                            # frame.copy(),
                            frame.get()
                            if rig_config["enable_gpu_processing"]
                            else frame,  # might be very slow if using gpu
                            project_directory + experiment_name + "/video/processed/" + current_time + ".jpg",
                        )
                    )

                frame_number += 1

                # run all LED stimuli if any of the arenas were rewarded
                if len(rewarded) > 0:
                    led.run_accumulated_led_stimulus()

                # process wait period
                if not started and time.time() - tracking_start_time > wait_period:
                    started = True
                    printer.print("Wait Period Over. Started Tracking.")
                elif not started:
                    printer.print(
                        "\rWait Period is active for allocating space in memory. Please wait for {:0.1f} seconds.".format(
                            wait_period - (time.time() - tracking_start_time)
                        )
                    )
                    time.sleep(1)

                # find all incomplete arenas
                incomplete_arenas = [i for i in fly_arenas if not trackers[i].completed]

                if rig_config["email_notifications"]:
                    # send email when 50% of the arenas are complete
                    if len(incomplete_arenas) <= len(fly_arenas) // 2 and len(incomplete_arenas) > 0 and not halfway_email_sent:
                        # send email
                        email_queue.put(
                            (
                                "Halfway through experiment. Currently trial numbers are: "
                                + ", ".join([str(trackers[i].trial_count+1) for i in fly_arenas])
                                + ". Please determine if the experiment should be prematurely terminated.",
                                "Experiment Halfway Through at " + current_time,
                            )
                        )
                        halfway_email_sent = True

                    # send email when all but one arena is complete
                    if len(incomplete_arenas) == 1 and not last_fly_email_sent:
                        # send email
                        email_queue.put(
                            (
                                "Experiment is almost complete. Currently trial numbers are: "
                                + ", ".join([str(trackers[i].trial_count+1) for i in fly_arenas])
                                + ". Please determine if the experiment should be prematurely terminated.",
                                "Experiment Almost Complete at " + current_time,
                            )
                        )
                        last_fly_email_sent = True

                if len(incomplete_arenas) == 0 or time.time() - tracking_start_time > maximum_time + wait_period:
                    # all arenas are complete
                    printer.print("All arenas complete.")
                    # send email
                    if rig_config["email_notifications"]:
                        email_queue.put(
                            (
                                "All arenas completed successfully. Please turn off the air valves and proceed to the next experiment.",
                                "Experiment Complete at " + current_time,
                            )
                        )
                    experiment_ongoing = False

            except KeyboardInterrupt:
                printer.print("Experiment interrupted.")
                # send email
                if rig_config["email_notifications"]:
                    email_queue.put(
                        (
                            "Experiment interrupted on purpose. Please turn off the air valves and proceed to the next experiment.",
                            "Experiment Interrupted at " + current_time,
                        )
                    )
                experiment_ongoing = False

            except Exception as e:
                printer.print("Error: " + str(e))
                # send email
                if rig_config["email_notifications"]:
                    email_queue.put(
                        (
                            "Experiment failed due to Error: "
                            + str(e)
                            + ". Please turn off the air valves and proceed to the next experiment.",
                            "Experiment Failed at " + current_time,
                        )
                    )
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

        # flip all valves to air
        printer.print("Flipping all odor valves to air...")
        for i in range(16):
            odor.publish(i, [0, 0, 0])
        printer.print("All odor valves flipped to air.")

        # wait for save queue to empty
        if rig_config["live_stream"]:
            printer.print("Waiting for save queue to empty...")
            save_queue.join()
            printer.print("Saved all frames.")

    # add a parallel thread to send emails every 5 minutes
    if rig_config["email_notifications"]:
        reminder_thread = threading.Thread(target=async_reminder, args=(300, email_queue))
        reminder_thread.start()

    # ask the user if they turned off the air supply
    printer.print("Please turn off the air supply and MFCs.")
    while True:
        air_supply = input("Did you turn off the air supply and MFCs? (y/n) ")
        if air_supply == "y" or air_supply == "Y":
            break
        elif air_supply == "n" or air_supply == "N":
            printer.print("Please turn off the air supply and MFCs and try again.")
        else:
            printer.print("Please enter either 'y' or 'n'.")

    # wait for email queue to empty
    if rig_config["email_notifications"]:
        printer.print("Waiting for email queue to empty...")
        email_queue.join()
        # reminder_thread.join()
        printer.print("Sent all emails.")

    # final message
    printer.print("Experiment complete. Thank you for using 16Y-Maze Rig for your experiment.")

    # close the printer
    printer.close()

    # end python script
    sys.exit()

