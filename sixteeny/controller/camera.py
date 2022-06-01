import PySpin

from PyQt5 import QtCore, QtGui, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import pickle, threading, queue
import matplotlib.pyplot as plt
import numpy as np
import cupy as cp
import time

_SYSTEM = None

class CameraError(Exception):
    """
    Exception raised when an error occurs in a camera.
    """
    pass

def list_cameras():
    """
    Return a list of connected Spinnaker cameras. Also initializes the PySpin `System`, if needed.
    """
    global _SYSTEM
    if _SYSTEM is None:
        _SYSTEM = PySpin.System.GetInstance()
    return _SYSTEM.GetCameras()

def video_writer(save_queue, writer):
    """
    Write images to disk.
    """
    while True:
        image = save_queue.get()
        if image is None:
            break
        else:
            writer.writeFrame(image)
            save_queue.task_done()

def video_player(play_queue, temp_folder):
    """
    Plot imaged from queue.
    """
    while True:
        image = play_queue.get()
        if image is None:
            break
        else:
            # save image to disk
            np.save(temp_folder + 'image.npy', image)
            play_queue.task_done()

class SpinnakerCamera:
    """
    A class used to encapsulate a Spinnaker camera
    
    Variables:
        cam : Camera object (PySpin.Camera)
        running : True if acquiring images (bool)
        initialized : True if the camera has been initialized (bool)
    
    Methods:
        init : Initializes the camera.  Automatically called if the camera is opened using a `with` clause.
        close : Closes the camera and cleans up.  Automatically called if the camera is opening using a `with` clause.
        start : Start recording images.
        stop : Stop recording images.
        get_raw_image : Get an raw image from the camera.
        get_array : Get an image from the camera, and convert it to a numpy/cupy array.
    """

    def __init__(
                self, 
                index=0, 
                gpu_enabled=True,
                CAMERA_FORMAT='Mono8',
                EXPOSURE_TIME=15000, 
                GAIN=10, 
                GAMMA=1, 
                MAX_FRAME_RATE=100,
                record_video=False, 
                video_output_path=None, 
                video_output_name=None, 
                show_video=False, 
                show_every_n=None,
                ffmpeg_path='C:\\ffmpeg\\bin\\',
                ):
        """
        Initialize the camera object.

        Variables:
            index : camera index (int)
            gpu_enabled : If True, use GPU acceleration. (bool)
            CAMERA_FORMAT : Format of the camera image. (str)
            EXPOSURE_TIME : Exposure time in microseconds. (int)
            GAIN : Gain in dB. (int)
            GAMMA : Gamma value. (int)
            MAX_FRAME_RATE : Maximum frame rate. (int)
            record_video : If True, record a video. (bool)
            video_output_path : Path to save the video. (str)
            video_output_name : Name of the video. (str)
            show_video : If True, show the video. (bool)
            show_every_n : If not None, show every nth frame. (int)
            ffmpeg_path : Path to ffmpeg. (str)
        """
        import skvideo
        skvideo.setFFmpegPath(ffmpeg_path)
        import skvideo.io

        self.initialized = False
        self.gpu_enabled = gpu_enabled

        self.record_video = record_video
        if self.record_video:
            assert video_output_path is not None, "video_output_path must be specified if record_video is True"
            assert video_output_name is not None, "video_output_name must be specified if record_video is True"
            self.video_output_path = video_output_path
            self.video_output_name = video_output_name
        
        self.show_video = show_video
        if self.show_video:
            assert show_every_n is not None, "show_every_n must be specified if show_video is True"
            self.show_every_n = show_every_n
        
        self.CAMERA_FORMAT = CAMERA_FORMAT
        self.EXPOSURE_TIME = EXPOSURE_TIME
        self.GAIN = GAIN
        self.GAMMA = GAMMA
        self.MAX_FRAME_RATE = MAX_FRAME_RATE

        self.time_of_last_frame = None

        cam_list = list_cameras()
        if not cam_list.GetSize():
            raise CameraError("No cameras detected.")
        if isinstance(index, int):
            self.cam = cam_list.GetByIndex(index)
        elif isinstance(index, str):
            self.cam = cam_list.GetBySerial(index)
        cam_list.Clear()

        self.running = False

    def init(self):
        """
        Initializes the camera setup.
        """
        self.cam.Init()

        # load default attributes
        self.cam.UserSetSelector.SetValue(PySpin.UserSetSelector_Default)
        self.cam.UserSetLoad()

        # set acquisition mode to continuous
        self.cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)

        # turn off auto exposure and set exposure time
        self.cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
        self.cam.ExposureTime.SetValue(self.EXPOSURE_TIME)

        # turn off auto gain and set gain
        self.cam.GainAuto.SetValue(PySpin.GainAuto_Off)
        self.cam.Gain.SetValue(self.GAIN)

        # set Gamma value
        self.cam.GammaEnable.SetValue(True)
        self.cam.Gamma.SetValue(self.GAMMA)

        # set pixel format
        if self.CAMERA_FORMAT == 'Mono8':
            self.cam.PixelFormat.SetValue(PySpin.PixelFormat_Mono8)
        elif self.CAMERA_FORMAT == 'Mono16':
            self.cam.PixelFormat.SetValue(PySpin.PixelFormat_Mono16)

        if self.record_video:
            self.timestamps = []
            if self.gpu_enabled:
                self.writer = skvideo.io.FFmpegWriter(self.video_output_path + self.video_output_name + '.mp4', outputdict={'-vcodec': 'h264'})
            else:
                self.writer = skvideo.io.FFmpegWriter(self.video_output_path + self.video_output_name + '.mp4', outputdict={'-vcodec': 'mpeg4'})
            self.save_queue = queue.Queue()
            self.save_thread = threading.Thread(target=video_writer, args=(self.save_queue, self.writer))
        
        if self.show_video:
            self.play_queue = queue.Queue()
            self.frame_count = 0
            self.play_thread = threading.Thread(target=video_player, args=(self.play_queue, 'temp/'))
        
        self.initialized = True

    def __enter__(self):
        """
        Initializes the camera setup using a context manager.
        """
        self.init()
        return self

    def close(self):
        """
        Closes the camera and cleans up.
        """

        self.stop()
        del self.cam
        self.initialized = False

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Closes the camera and cleans up using a context manager.
        """
        self.close()

    def start(self):
        """
        Start image acquisition.
        """
        if not self.running:
            self.cam.BeginAcquisition()
            if self.record_video:
                self.save_thread.start()
            if self.show_video:
                self.play_thread.start()
            self.running = True
            self.time_of_last_frame = time.time()

    def stop(self):
        """
        Stop image acquisition.
        """
        if self.running:
            self.cam.EndAcquisition()
            if self.record_video:
                self.save_queue.join()
                self.writer.close()
                with open(self.video_output_path + self.video_output_name + '_timestamps.pkl', 'wb') as f:
                    pickle.dump(self.timestamps, f)
            if self.show_video:
                self.play_queue.join()
                self.frame_count = 0
        self.running = False

    def get_raw_image(self, wait=True):
        """
        Get a raw image from the camera.

        Variables:
            wait : If True, wait for an image to be acquired.  Throw an error if no image is available. (bool)
        
        Returns:
            image : Image from the camera (PySpin.Image)
        """
        return self.cam.GetNextImage(PySpin.EVENT_TIMEOUT_INFINITE if wait else PySpin.EVENT_TIMEOUT_NONE)

    def get_array(self, wait=True, get_chunk=False):
        """
        Get an image from the camera, and convert it to a NumPy/CuPy array.

        Variables:
            wait : If True, wait for an image to be acquired.  Throw an error if no image is available. (bool)
            get_chunk : If True, return chunk data (bool)

        Returns:
            image : Image from the camera (numpy.ndarray/cupy.ndarray)
            chunk : PySpin chunk data (PySpin)
        """
        time_since_last_frame = time.time() - self.time_of_last_frame
        if time_since_last_frame < 1.0 / self.MAX_FRAME_RATE:
            time.sleep(max(1.0 / self.MAX_FRAME_RATE - time_since_last_frame, 0))
        
        img = self.get_raw_image(wait)
        
        self.time_of_last_frame = time.time()

        dtype = np.uint8 if self.CAMERA_FORMAT == 'Mono8' else np.uint16
        arr = np.array(img.GetData(), dtype=dtype).reshape(img.GetHeight(), img.GetWidth())
        
        if self.record_video:
            self.timestamps.append(img.GetTimeStamp())
            self.save_queue.put(arr)
        
        if self.show_video:
            if self.frame_count % self.show_every_n == 0:
                self.play_queue.put(arr)
            self.frame_count += 1

        if self.gpu_enabled:
            dtype = cp.uint8 if self.CAMERA_FORMAT == 'Mono8' else cp.uint16
            arr = cp.array(arr, dtype=dtype)
        
        if get_chunk:
            return arr, img.GetChunkData()
        else:
            return arr


