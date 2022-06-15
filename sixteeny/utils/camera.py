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
    while timestamps[-1] - timestamps[0] < time_to_record:
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

def change_in_image(current_image, previous_image, relative=False, gpu_enabled=False):
    """
    Calculate change in image keeping only positive values.

    Variables:
        current_image (numpy.ndarray/cupy.ndarray): current image
        previous_image (numpy.ndarray/cupy.ndarray): previous image
    
    Returns:
        del_image (numpy.ndarray/cupy.ndarray): change in image
    """
    if gpu_enabled:
        if relative:
            del_image = cp.maximum(current_image - previous_image, 0) / cp.maximum(previous_image, 1)
        else:
            del_image = cp.maximum(current_image - previous_image, 0)
    else:
        if relative:
            del_image = np.maximum(current_image - previous_image, 0) / np.maximum(previous_image, 1)
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

def combine_binarized_images(image1, image2, logical_operator, gpu_enabled=False):
    """
    Combine two binarized images using a logical operator.

    Variables:
        image1 (numpy.ndarray/cupy.ndarray): first image
        image2 (numpy.ndarray/cupy.ndarray): second image
        logical_operator (str): logical operator to use (and, or, xor)
        
    Returns:
        combined_image (numpy.ndarray/cupy.ndarray): combined image
    """
    if gpu_enabled:
        if logical_operator == 'and':
            combined_image = cp.logical_and(image1, image2)
        elif logical_operator == 'or':
            combined_image = cp.logical_or(image1, image2)
        elif logical_operator == 'xor':
            combined_image = cp.logical_xor(image1, image2)
        else:
            raise ValueError('logical_operator must be one of "and", "or", or "xor"')
    else:
        if logical_operator == 'and':
            combined_image = np.logical_and(image1, image2)
        elif logical_operator == 'or':
            combined_image = np.logical_or(image1, image2)
        elif logical_operator == 'xor':
            combined_image = np.logical_xor(image1, image2)
        else:
            raise ValueError('logical_operator must be one of "and", "or", or "xor"')
    return combined_image
