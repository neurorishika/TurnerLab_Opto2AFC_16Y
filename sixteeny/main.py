import numpy as np
# import cupy as cp
import matplotlib.pyplot as plt

from sixteeny.utils.tracker import ArenaTracker
from sixteeny.utils.camera import record_background
from sixteeny.controller.camera import SpinnakerCamera
from sixteeny.controller.odor import OdorValveController
from sixteeny.controller.led import LEDController
from sixteeny.controller.mfc import MFCController
from sixteeny.gui.MFC_viewer import start_gui

import sys
import os
import json
import time

import threading

if __name__ == '__main__':
    # Get project directory and name from command line arguments
    if len(sys.argv) > 2:
        project_directory = sys.argv[1]
        experiment_name = sys.argv[2]
    else:
        # send error message
        print('Please specify experiment directory and name as command line arguments.')
        sys.exit(1)
    
    # ensure that project directory exists
    if project_directory[-1] != '/':
        project_directory += '/'
    if not os.path.isdir(project_directory):
        print('Project directory does not exist.')
        sys.exit(1)

    # ensure that experiment name directory exists
    if not os.path.isdir(project_directory + experiment_name):
        print('Experiment directory does not exist.')
        sys.exit(1)

    # load the rig configuration file
    if not os.path.isfile(project_directory + experiment_name + '/config.yarena'):
        print('Experiment config file does not exist.')
        sys.exit(1)

    with open(project_directory + experiment_name + '/config.yarena', 'r') as f:
        rig_config = json.load(f)

    # find all *.yexperiment files in the folder
    experiment_files = [f for f in os.listdir(project_directory + experiment_name) if f.endswith('.yexperiment')]

    n_flies = len(experiment_files)
    fly_arenas = [experiment.split('.')[0].split('_')[1] for experiment in experiment_files]

    # Initialize controllers
    controllers = {}
    with SpinnakerCamera(
        index=rig_config['camera_index'],
        gpu_enabled=rig_config['enable_gpu_processing'],
        CAMERA_FORMAT=rig_config['pixel_format'],
        EXPOSURE_TIME=rig_config['exposure_time'],
        GAIN=rig_config['gain'],
        GAMMA=10,
        MAX_FRAME_RATE=rig_config['max_frame_rate'],
        record_video=rig_config['record_video'],
        video_output_path= rig_config['video_folder'],
        video_output_name=experiment_name+'_'+str(rig_config['camera_index']),
        show_video=rig_config['show_video'],
        show_every_n=1,
        ffmpeg_path=rig_config['ffmpeg_path'],
    ) as camera, \
    OdorValveController(
        minimum_delay=rig_config['minimum_message_delay']
    ) as odor, \
    LEDController(
        ports=rig_config['com_ports'], 
        baudrate=rig_config['baud_rate'], 
        arena_panel_ids=rig_config['quadrant_ids']
    ) as led, \
    MFCController(
        com_port=rig_config['mfc_com_port'], 
        device_ids=rig_config['mfc_device_ids'],
        default_flow_rate=rig_config['mfc_flow_rate'],
        default_gas_type=rig_config['mfc_gas_type']
    ) as mfc:
        controllers['camera'] = camera
        controllers['odor'] = odor
        controllers['led'] = led
        controllers['mfc'] = mfc

        # Start IR backlight
        led.turn_on_backlight(rig_config['ir_intensity'])

        # Start MFC Viewer in a separate thread
        mfc_viewer_thread = threading.Thread(target=start_gui, args=(controllers,))
        mfc_viewer_thread.start()

        # record the background image
        background,eff_fps,eff_duration,timestamp = record_background(
                                                time_to_record=rig_config['background_record_time'],
                                                camera=camera,
                                                gpu_enabled=rig_config['enable_gpu_processing'],
                                            )



    

    
    