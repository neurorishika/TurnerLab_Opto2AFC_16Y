import numpy as np
import skimage.morphology as skm
import cupy as cp
import cucim.skimage.morphology as cskm
from controller.camera import SpinnakerCamera
import time

def record_background(time_to_record, camera, gpu_enabled=False):
    """
    Record a background image averaged over a given time.

    Variables:
        time_to_record (float): time to record in seconds
        camera (SpinnakerCamera): camera object
        gpus_enabled (bool): whether or not to use GPU
    
    Returns:
        background (numpy.ndarray/cupy.ndarray): background image
        eff_fps (int): effective frames per second
        eff_duration (float): effective duration
    """
    camera.start(dont_record=True)
    images = []
    timestamps = [time.time()]
    while timestamps[-1]-timestamps[0] < time_to_record:
        images.append(camera.get_array())
        timestamps.append(time.time())
    camera.stop(dont_record=True)
    if gpu_enabled:
        background = cp.mean(cp.array(images), axis=0)
    else:
        background = np.mean(images, axis=0)
    eff_fps = 1 / np.mean(np.diff(timestamps))
    eff_duration = timestamps[-1] - timestamps[0]
    return background, eff_fps, eff_duration, timestamps

def background_subtraction(image, background, gpu_enabled=False):
    """
    Subtract background from image keeping only positive values.

    Variables:
        image (numpy.ndarray/cupy.ndarray): image to subtract background from
        background (numpy.ndarray/cupy.ndarray): background image
    
    Returns:
        subtracted_image (numpy.ndarray/cupy.ndarray): subtracted image
    """
    if gpu_enabled:
        subtracted_image = cp.maximum(image - background, 0)
    else:
        subtracted_image = np.maximum(image - background, 0)
    return subtracted_image

def change_in_image(current_image, previous_image, gpu_enabled=False):
    """
    Calculate change in image keeping only positive values.

    Variables:
        current_image (numpy.ndarray/cupy.ndarray): current image
        previous_image (numpy.ndarray/cupy.ndarray): previous image
    
    Returns:
        del_image (numpy.ndarray/cupy.ndarray): change in image
    """
    if gpu_enabled:
        del_image = cp.maximum(current_image - previous_image, 0)
    else:
        del_image = np.maximum(current_image - previous_image, 0)
    return del_image

def binarize(image, threshold, gpu_enabled=False):
    """
    Binarize image to boolean array.

    Variables:
        image (numpy.ndarray/cupy.ndarray): image to binarize
        threshold (int): threshold value
    
    Returns:
        binarized_image (numpy.ndarray/cupy.ndarray): binarized image
    """
    if gpu_enabled:
        binarized_image = cp.greater(image, threshold)
    else:
        binarized_image = np.greater(image, threshold)
    return binarized_image

def clean_objects(current_image, previous_image, background_image, threshold, closing_radius, gpu_enabled=False):
    """
    Clean objects from image.
    
    Variables:
        current_image (numpy.ndarray/cupy.ndarray): current
        previous_image (numpy.ndarray/cupy.ndarray): previous
        background_image (numpy.ndarray/cupy.ndarray): background
        threshold (int): threshold value
    
    Returns:
        cleaned_image (numpy.ndarray/cupy.ndarray): cleaned image
    """
    del_image = change_in_image(current_image, previous_image, gpu_enabled)
    binarized_change_in_image = binarize(del_image, threshold, gpu_enabled)
    contrasted_image = background_subtraction(current_image, background_image, gpu_enabled)
    binarized_contrasted_image = binarize(contrasted_image, threshold, gpu_enabled)
    cleaned_image = binarized_change_in_image | binarized_contrasted_image
    # if gpu_enabled:
    #     cleaned_image = cskm.binary_closing(cleaned_image, cskm.disk(closing_radius))
    # else:
    #     cleaned_image = skm.binary_closing(cleaned_image, skm.disk(closing_radius))
    return cleaned_image